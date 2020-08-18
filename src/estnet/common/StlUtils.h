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

#ifndef __UTILS_STL_UTILS_H__
#define __UTILS_STL_UTILS_H__

#include <algorithm> // std::find
#include <map>       // std::map
#include <vector>    // std::vector
#include <set>       // std::set

namespace estnet {

/** @brief checks if vector contains an item */
template<typename T> bool contains(const std::vector<T> &vector,
        const T& item) {
    return std::find(vector.begin(), vector.end(), item) != vector.end();
}

/** @brief checks if map contains given key */
template<typename K, typename V>
bool contains(const std::map<K, V> &map, const K& key) {
    return map.find(key) != map.end();
}

/** @brief checks if set contains given key */
template<typename K>
bool contains(const std::set<K> &set, const K& key) {
    return set.find(key) != set.end();
}

/** @brief puts key and value into the map if key doesn't
 *  already exist, if it does throws an exception */
template<typename K, typename V>
void emplaceOrThrow(std::map<K, V> &map, K key, V value) {
    if (contains(map, key)) {
        throw std::invalid_argument("Key already in map");
    }
    map.emplace(key, value);
}

/** @brief convenience helper for intersection between two sets */
template<typename T>
std::set<T> getSetIntersection(const std::set<T>& lhs, const std::set<T>& rhs) {
    std::set<T> res;
    // sets are already ordered, no need to order them explicitly before set_intersection
    std::set_intersection(lhs.begin(), lhs.end(), rhs.begin(), rhs.end(),
            std::inserter(res, res.begin()));
    return res;
}

/** @brief convenience helper for intersection between a sets and a potential element of the set */
template<typename T>
std::set<T> getSetElementIntersection(const std::set<T>& lhs,
        const T& rhsElement) {
    std::set<T> rhs;
    rhs.insert(rhsElement);
    return getSetIntersection(lhs, rhs);
}

/** @brief replaces toReplace with replacement in strin str
 * inline so that it can be defined by multiple compilation units */
inline bool stringReplace(std::string& str, const std::string& toReplace,
        const std::string& replacement) {
    size_t start_pos = str.find(toReplace);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, toReplace.length(), replacement);
    return true;
}

}  // namespace estnet

#endif
