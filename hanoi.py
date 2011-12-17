#!/usr/bin/env python
# 
# hanoi.py - an exhaustive tree search implementation of the Towers of Hanoi
# Copyright (C) 2011 Mike Miller
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

class Hanoi(object):

    """Construct and search a game tree to solve the Towers of Hanoi."""

    def __init__(self, n):
        """Initialize a Towers of Hanoi game with n disks."""
        self.all_disks = (1 << n) - 1
        self.top_disk = 1 << (n - 1)
        self.init_state = (self.all_disks, 0, 0)
        self.goal_state = (0, 0, self.all_disks)
        self.visited_states = set()
        self.visited_states.add(self.init_state)
        self.tree = self.construct_game_tree(self.init_state)

    def __len__(self):
        return len(self.visited_states)

    def construct_game_tree(self, state):
        """Construct a game tree with the given initial state."""
        root = node(state)
        for next in self.possible_states(state):
            if not next in self.visited_states:
                self.visited_states.add(next)
                root.children.append(self.construct_game_tree(next))
        return root

    def possible_states(self, state):
        """Return a list of possible successor states from the given state."""
        nexts = []
        pegs = range(0, len(state))
        for peg in pegs:
            if state[peg]:
                other_pegs = pegs[:]
                other_pegs.remove(peg)
                for opeg in other_pegs:
                    next = self.try_move(state, peg, opeg)
                    if next:
                        nexts.append(next)
        return nexts

    def try_move(self, state, i, j):
        """Attempt to move a disk, returning the next state if legal."""
        if state[j] < state[i]:
            # There is a legal move, find the topmost disk in i
            disk = self.top_disk
            while not disk & state[i]:
                disk >>= 1
            next = list(state)
            next[i] &= ~disk
            next[j] |= disk
            return tuple(next)
        return None

class node(object):

    """A state node container that encodes the state and references to
       successor states.
    """

    def __init__(self, state, children=[]):
        """Initialize a node with the given state and successor nodes."""
        self.state = state
        self.children = children

if __name__ == '__main__':
    h = Hanoi(4)
    print "Size of the game tree: %d" % len(h)
