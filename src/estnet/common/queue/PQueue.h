//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __UTILS_PQUEUE_H__
#define __UTILS_PQUEUE_H__

#include <algorithm> // push_heap, pop_heap
#include <vector>

namespace estnet {

/** an implementation of a priority queue
 * with copy access
 */
template <typename Type, typename Compare = std::less<Type>> class pqueue {
private:
  Compare _compare;
  std::vector<Type> _elements;

public:
  explicit pqueue(Compare compare = Compare()) : _compare{std::move(compare)} {}

  /** @brief add element into priority queue */
  void push(Type element) {
    _elements.push_back(element);
    std::push_heap(_elements.begin(), _elements.end(), _compare);
  }
  /** @brief remove bottom element from priority queue */
  Type pop() {
    std::pop_heap(_elements.begin(), _elements.end(), _compare);
    Type result = _elements.back();
    _elements.pop_back();
    return result;
  }
  /** @brief returns top element from priority queue */
  Type top() const { return _elements.front(); }
  /** @brief returns copy of the priority queu as a vector */
  std::vector<Type> copy() const {
    std::vector<Type> cpy = _elements;
    return cpy;
  }
  /** @brief returns true if priority queue is empty */
  bool empty() const { return _elements.empty(); }
  /** @brief accesses element at index n */
  const Type &operator[](unsigned int n) const { return _elements[n]; }
  /** @brief accesses element at index n */
  const Type &at(unsigned int n) const { return _elements.at(n); }
  /** @brief removes all elements from priority queue */
  void clear() { _elements.clear(); }
};

}  // namespace estnet

#endif
