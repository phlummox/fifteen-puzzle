
// compiler requirements: requires C++11.
// compiles with clang++ and g++ on linux.

#define NDEBUG 

#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <array>
#include <utility>
#include <stdexcept>
#include <iostream> 
#include <sstream> 
#include <initializer_list>
#include <cassert>
#include <string>
#include <algorithm>
#include <vector>

using std::string;
using std::endl;
using std::cout;
using std::vector;

// type used for puzzle cells.
typedef unsigned char uchar;

// an (x,y) position
typedef std::pair<uchar,uchar> pos_t;

//const bool DEBUG = false;

// typesafe enum for directions (Up, Down, Left, Right).
//
// taken from wikibook example on typesafe enums.
// doesn't seem to add any significant overhead,
// so may as well leave it in.

template<typename def, typename inner = typename def::type>
class safe_enum : public def
{
  typedef inner type;
  inner val;

public:

  safe_enum() {}
  safe_enum(type v) : val(v) {}
  type underlying() const { return val; }

  friend bool operator == (const safe_enum & lhs, const safe_enum & rhs) { return lhs.val == rhs.val; }
  friend bool operator != (const safe_enum & lhs, const safe_enum & rhs) { return lhs.val != rhs.val; }
  friend bool operator <  (const safe_enum & lhs, const safe_enum & rhs) { return lhs.val <  rhs.val; }
  friend bool operator <= (const safe_enum & lhs, const safe_enum & rhs) { return lhs.val <= rhs.val; }
  friend bool operator >  (const safe_enum & lhs, const safe_enum & rhs) { return lhs.val >  rhs.val; }
  friend bool operator >= (const safe_enum & lhs, const safe_enum & rhs) { return lhs.val >= rhs.val; }
};


struct direction_def {
  enum type { Up, Down, Left, Right };
};
typedef safe_enum<direction_def> dir;

dir charToDir(char c) {
  switch(c) {
     case 'u'  :
        return dir::Up;
     case 'd'  :
        return dir::Down;
     case 'l'  :
        return dir::Left;
     case 'r'  :
        return dir::Right;
     default : 
      throw std::logic_error( "charToDir bad val" );
  }
}



dir opposite(const dir & d) {
  if (d == dir::Up ) {
    return dir::Down;      
  } else if (d == dir::Down) {
    return dir::Up;      
  } else if (d == dir::Left) {
    return dir::Right;      
  } else if (d == dir::Right) {
    return dir::Left;      
  } else {
      throw std::logic_error( "impossible d" );
  }
}

string to_string(const dir &d) {
  if (d == dir::Up ) {
    return "u";      
  } else if (d == dir::Down) {
    return "d";      
  } else if (d == dir::Left) {
    return "l";      
  } else if (d == dir::Right) {
    return "r";      
  } else {
      throw std::logic_error( "impossible d" );
  }
}

std::ostream& operator<< (std::ostream& stream, const dir& d) {
  stream << to_string(d);
  return stream; 
}

// class for fifteen-puzzle.
// Numbers are stored in a 4x4 std::array.
// There's also a lookup table from each piece to its position in the grid
// (pieceToIdx).
// No doubt this could be made more efficient, but seems fine for the mo.
// For use when storing in a container: we generate the /compressed_val/,
//   which is just the cell values, all squeezed into a single uint64_t
//   (since cell values never go beyond 4 bits,
//   so an 8 byte uint just holds them all.)
//   The /compressed_val/ is calculated once, when needed, and cached. 

// changes to puzzle state are made in a "functional" style -
// puzzles are treated as more or less immutable, and methods
// like 'moveBlankTo' return a fresh /puzzle/ object.

class puzzle {
public:
  static const uchar SIZE = 4;
  static const uchar BLANK = 0;

private:
  typedef std::array<uchar, SIZE> row;
  
  std::array<row, SIZE> cells; // 
  std::array<uchar, SIZE*SIZE> pieceToIdx; 
  mutable uint64_t compressed_val; // cached, compressed version of
                           // data. rather than creating copy constructor
                           // and assignment op, we rely on 
                           // fact that only moveBlankTo() alters data.
                           // (of the fresh puzzle, not existing).

public:
  // calculate the /compressed_val/
  inline uint64_t to_int() const {
    if (compressed_val != 0) {
      return compressed_val;
    }

    uint64_t res = 0;
    for (int y = 0; y < SIZE; y++) {
      for (int x = 0; x < SIZE; x++) {
        if (y != 0 || x != 0) { res = res << 4; };
        uchar curr_ch = cells[y][x]; // & 0xf; // can only be up to 0xf anyway...
        //assert ( cells[y][x] == curr_ch) ;
        res += curr_ch; 
      }
    }

    compressed_val = res;
    return res;
  }


#if 0
  // get rid of the assignment operator
  puzzle& operator=(const puzzle& other) {
    // check for self-assignment
    if(&other == this) {
      throw std::logic_error( "assigned puzzle to self" );
    }

    cells = other.cells;
    pieceToIdx = other.pieceToIdx;
    compressed_val = 0;
    return *this;
  } 

#endif
  // copy constructor.
  // used in moveBlankTo().
  puzzle(const puzzle& other)
    : cells(other.cells)
      , pieceToIdx(other.pieceToIdx)
      , compressed_val(0)
  { }

  // constructor
  // usage:
  //    puzzle p { '_', { 1, 2, ....  15, 0 }};
  // 
  // The "char" was just to distinguish this from other constructors,
  // which have since been deleted.
  puzzle( char c, std::initializer_list<uchar> li) 
    : compressed_val(0)
  {
    if ( li.size() != SIZE * SIZE) {
      std::ostringstream os;
      os << "expected init list of size " << ((int)SIZE * SIZE) << ", got " << (li.size());
      throw std::logic_error( os.str() );
    }

    uchar idx = 0;

    for (auto && piece : li) {
      int pieceX = idxToX(idx);     
      int pieceY = idxToY(idx);  
      cells[pieceY][pieceX] = piece;
      pieceToIdx[piece] = idx;
      idx++;
    }

#ifdef DEBUG
    assert( isConsistent() );
#endif
  }  

  static inline int idxToX(int idx) {
    return idx % SIZE;
  }
  
  static inline int idxToY(int idx) {
    return idx / SIZE;
  }

  static inline int posToIdx(int x, int y) {
    return (y * SIZE) + x;
  }

  inline pos_t getBlankPos() const {
    uchar blankIdx = pieceToIdx.at(BLANK);
    int blankX = idxToX(blankIdx);
    int blankY = idxToY(blankIdx);
    return std::make_pair(blankX, blankY);
  }

  inline uchar pieceAtPos(int x, int y) const {
    return cells.at(y).at(x);
  }

  // returns fresh puzzle
  inline puzzle moveBlankTo(int tgtX, int tgtY) const {
#ifdef DEBUG
// or could define NDEBUG or whatever
    assert( tgtX >= 0 && tgtX < SIZE );
    assert( tgtY >= 0 && tgtY < SIZE );
#endif

    // ** tgtPos = tgtX, tgtY
    uchar blankIdx = pieceToIdx[BLANK];
    int blankX = idxToX(blankIdx);
    int blankY = idxToY(blankIdx);
    
    uchar tgtPiece = pieceAtPos(tgtX, tgtY);
    
    puzzle newP {  *this } ; 
    // ** newP.cells
    newP.cells.at(blankY).at(blankX) = tgtPiece;
    newP.cells.at(tgtY).at(tgtX) = BLANK;
    
    // ** newP.valLookup
    newP.pieceToIdx.at(BLANK) = posToIdx(tgtX, tgtY);
    newP.pieceToIdx.at(tgtPiece) = blankIdx;

    // re-set 
    newP.compressed_val = 0;

#ifdef DEBUG
    assert ( newP.isConsistent() ); 
#endif
    return newP;
  }

  // use for debugging - check that /cells/ and /pieceToIdx/
  // are in sync
  bool isConsistent() const {
    for(uchar i = 0; i < SIZE*SIZE; i++) {
      uchar piece = i;
      uchar pieceIdx = pieceToIdx[piece];
      int pieceX = idxToX(pieceIdx);
      int pieceY = idxToY(pieceIdx);

      bool same = cells.at(pieceY).at(pieceX) == piece;
      if (!same) {
#ifdef DEBUG
        std::ostringstream os;
        os << "piece " << ((int)i) << " should be at idx " << ((int)pieceIdx) << ", but found " << ((int)cells[pieceY][pieceX]) ;
        throw std::logic_error( os.str() );
#endif
        return false;
      }
    }
    return true;
  }

  string to_string() const {
    std::ostringstream os;
    for (int y = 0; y < SIZE; y++) {
      for (int x = 0; x < SIZE; x++ ) {
        uchar piece = pieceAtPos(x,y);
        char str[3];
        snprintf(str, 3, "%2d", (int) piece);
        os << str << " ";
      }
      if (y != SIZE) { os << endl; }
    }
    return os.str();
  }

  string dump() const {
      std::ostringstream os;
      os << "  cells: ";
      for (int y = 0; y < SIZE; y++) 
        for (int x = 0; x < SIZE; x++) 
          os << ((int)pieceAtPos(x,y)) << " ";
      os << endl;
      os << "  pieceToIdx: ";
      for (int i = 0; i < SIZE*SIZE; i++) 
        os << ((int)pieceToIdx[i]) << " ";
      os << endl;
      return os.str();
  }

  // returns fresh puzzle
  puzzle moveDir(const dir &dir) const {
    pos_t blankPos = getBlankPos();
    int blankX = blankPos.first;
    int blankY = blankPos.second;

    if (dir == dir::Up ) {
      return moveBlankTo(blankX, blankY - 1);      
    } else if (dir == dir::Down) {
      return moveBlankTo(blankX, blankY + 1);      
    } else if (dir == dir::Left) {
      return moveBlankTo(blankX - 1, blankY);      
    } else if (dir == dir::Right) {
      return moveBlankTo(blankX + 1, blankY);      
    } else {
        throw std::logic_error( "impossible dir" );
    }
  }

  // used for storage and retrieval in a /std::set/ or /std::map/
  inline bool operator< (const puzzle & other) const {
        return to_int() < other.to_int();
  };

  // puzzle ==
  inline bool operator==(const puzzle &other) const{
    bool res = to_int() == other.to_int();
    //bool res = true;
    return res;
  }


private:

  // debug only ... could remove
  void set(int x, int y, uchar val) {
    cells[y][x] = val;
  }
};

// the state we're aiming for.
// some solvers reduce the amount of code they need
// by insisting the blank wind up at (0,0), but that seems
// like a bit of a cheat, to me. Overly limits generality
// of the solver and makes it fiddly to alter.
static const puzzle solution 
  { '_', { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0 } };

// our heuristic for a-star search - we use number of mismatches.
// see <https://heuristicswiki.wikispaces.com/N+-+Puzzle> for more.
int num_mismatches(const puzzle &p1, const puzzle &p2) {
  int tot = 0;
  for (int y = 0; y < puzzle::SIZE; y++) {
    for (int x = 0; x < puzzle::SIZE; x++) {
      if ( p1.pieceAtPos(x,y) != p2.pieceAtPos(x,y) ) { tot ++; }
    }
  }
  return tot;
}

// for debugging, attempts to turn a number of
// "bogus" puzzle states (w/ only 1 tile, set to 1)
// to ints.
#if 0
void testToInt() {
  puzzle tmp { solution };

  for (int y = 0; y < puzzle::SIZE; y++) {
    for (int x = 0; x < puzzle::SIZE; x++) {
      tmp.set(x,y,0);
    }
  }

  for (int y = 0; y < puzzle::SIZE; y++) {
    for (int x = 0; x < puzzle::SIZE; x++) {
      puzzle tmpTmp { tmp };
      tmpTmp.set(x,y,1);
      cout << "tmpTmp\n" << tmpTmp.to_string() << endl;
      uint64_t asInt = tmpTmp.to_int();
      cout << "asInt: " << asInt << endl;
    }
  }
}
#endif

// we want to nicely print the back-chain of both
// puzzle states, AND the directions we moved the blank
// in each case. So we keep a tuple of /puzzle/ and /dir/.
typedef std::pair<puzzle, dir> puzAndDir;

inline bool operator< (const puzAndDir & a, const puzAndDir & b) {
      return a.first < b.first;
}

bool operator== (const puzAndDir & a, const puzAndDir & b) {
      return a.first == b.first;
}

std::ostream& operator<< (std::ostream& stream, const puzAndDir& pd) {
  stream << "(" << pd.first.to_string() << ", " << to_string(pd.second) ;
  return stream; 
}

//#define DEBUG
#include "astar.h"


namespace solver {

  // get neighbouring puzzle positions of (x,y) that
  // are still in the grid
  static vector<pos_t> grid_neighbours( int x, int y ) {
    vector<pos_t> res {  };
    // add { (x-1,y) , (x+1,y) }
    for( auto && newX : { x-1, x+1 } ) {
      if (newX >= 0 && newX < puzzle::SIZE)
              { res.push_back( std::make_pair(newX, y) ); }
    }
    // add { (x,y-1) , (x,y+1) }
    for( auto && newY : { y-1, y+1 } ) {
      if (newY >= 0 && newY < puzzle::SIZE)
              { res.push_back( std::make_pair(x, newY) ); }
    }
    return res; 
  }

  // solve a puzzle instance
  static void solve(const puzzle & start_puzzle) {
    // todo: the 'direction' inserted in the pair is bogus,
    // find a nicer way to do this. Could use the 'std::optional'
    // type, but that's only avail in C++14 at minimum.
    puzAndDir goal = std::make_pair(solution, dir::Up);
    puzAndDir start = std::make_pair(start_puzzle, dir::Up);

    // functions for: calculating heuristic, generating
    // neighbouring states, and calculating the hash of a state. 

    astar::heuristic<puzAndDir> myHeuristic = [&goal](const puzAndDir &p) {
      size_t res = num_mismatches(p.first, goal.first);
      return res;
    };

    astar::get_neighbours<puzAndDir> myGen = [](const puzAndDir &curr) {
      vector<puzAndDir> res;

      pos_t blankPos = curr.first.getBlankPos();
      int blankX = blankPos.first;
      int blankY = blankPos.second;
      vector<pos_t> neighbourPossies = grid_neighbours(blankX, blankY);

      for (auto && possie : neighbourPossies) {
        auto theDir = [&possie, &blankX, &blankY]() {
          if (possie.first == blankX - 1) 
            { return dir::Left; }
          else if (possie.first == blankX + 1) 
            { return dir::Right; }
          else if (possie.second == blankY - 1) 
            { return dir::Up; }
          else if (possie.second == blankY + 1) 
            { return dir::Down; }
          else {
            throw std::logic_error( "bad neighbour dir" );
          }
        }();
        puzzle neighbPuz = 
          curr.first.moveBlankTo(possie.first, possie.second);

        puzAndDir successor = std::make_pair(neighbPuz, theDir);
        res.push_back(successor);
      }

      return res;
    };

    astar::hash<puzAndDir> myHash = [](const puzAndDir &p) {
      size_t res = p.first.to_int();
      return res;
    };

    // do the search.
    vector<puzAndDir> search_results = 
      astar::astar_search<puzAndDir>( 
          start, goal, myGen, myHeuristic, std::equal_to<puzAndDir>(),
          //std::less<puzAndDir>(),
          myHash,
          10000000 // lim
          );

    cout << "direction chain: " << endl;
    for(auto && res : search_results ) {
        cout << res.second << " ";
    }
    cout << endl;
    cout << "states: " << endl;
    for(auto && res : search_results ) {
        cout << res.first.to_string() << endl; 
    }

  }

}

#if 0
template<typename T, typename Cmp>
inline bool contains(std::set<T, Cmp> & s, const T & el) {
  return s.find(el) != s.end();
} 
#endif

static const puzzle hard_puzzle { 'a', {
  15, 14,  1,  6,
   9, 11,  4, 12,
   0, 10,  7,  3,
  13,  8,  5,  2 }
};

// this is (one) solution to the 'hard' puzzle instance
// above. We test our code by applying this in reverse to the
// solution state, to get a problem instance of known difficulty.
static const string sol_path_str { 
  "rrrulddluuuldrurdddrullulurrrddldluurddlulurruldrdrd"
};

// reverse of sol_path_str, above. (i.e., backwards, and with
// directions reversed).
static const vector<dir> reverse_path = ([](){
  vector<dir> sol_path {};

  // add dir version of sol_path_str
  std::for_each(
      sol_path_str.begin(), 
      sol_path_str.end(), 
      [&sol_path](char c) {
        sol_path.push_back( charToDir(c) );
      }
  );

  // reverse it
  std::reverse( sol_path.begin(), sol_path.end() );

  // reverse each direction
  for ( auto it = sol_path.begin(); it != sol_path.end(); it++ ) {
    *it = opposite(*it);
  }

  return sol_path;
})();

// args: the number of moves "backwards" to go from the solution state,
// using sol_path_str, so we have a puzzle instance of known
// difficulty.
int main(int argc, char ** argv) {
  argc--;
  argv++;
  
  // default difficulty: 22 steps
  int max_i = 22;

  if (argc >= 1) {
    max_i = atoi (argv[0]);
    cout <<  "got param for max_i: " << max_i << endl;
  }

  try{

    puzzle cur { solution };

    cout << "sol:\n" << solution.to_string() << endl;

    // on my PC:
    // improvements to code brought n = 28 down from about 30 secs,
    // to 1.76 secs or so.
    for(int i = 0; i < max_i; i++) {
      auto d = reverse_path.at(i);
      cur = cur.moveDir(d);
    }

    cout << "starting state:" << endl;
    cout << cur.to_string() << endl; 

    cout << "\n...got starting pos now. try solve" << endl;

    solver::solve(cur);

  } catch (const std::exception& ex) {
    cout << "exception: " << ex.what() << endl;
  } catch (...) {
    cout << "unexpected exception" << endl;
  }

  return 0;
}


