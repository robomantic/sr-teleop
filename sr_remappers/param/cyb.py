#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
file = open("cyberglovetoshadowhand.map")

first = True
matrix = []

for line in file:
    if first:
        first = False
    else:
        matrix.append(line.strip("\n").split(" "))
file.close()

matrix_t = [ [matrix[i][col] for i in xrange(len(matrix))] for col in xrange(len(matrix[0])) ]

file = open("cyberglovetoshadowhand_transposed.map", "w")
for line in matrix_t:
    file.write(" ".join(line) + "\n")
file.close()



