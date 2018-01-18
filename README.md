## fifteen-puzzle

A fifteen-puzzle solver (see, <https://en.wikipedia.org/wiki/15_puzzle>) in C++11
using A-star search (see <https://en.wikipedia.org/wiki/A*_search_algorithm>) and a simple "no. of mismatches" heuristic.

The a-star implementation is pretty straightforward, and uses hash-sets to store visited and "to-visit" states/nodes.

## use

Should compile with any version of g++ or clang++ that supports C++11:

    make puzzle

You can test it on puzzle instances of known difficulty, by specifing a puzzle instance which is some
number of steps *n* from a solution, up to 53 (I think) - where 53 steps gives you to an instance
given on Rosetta Code [here](https://rosettacode.org/wiki/15_puzzle_solver), solving which requires ... idk,
some godawful number of states.

    ./puzzle 10


