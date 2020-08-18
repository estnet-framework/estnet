//
// Copyright (C) 2020 Computer Science VII: Robotics and Telematics - 
// Julius-Maximilians-Universitaet Wuerzburg
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

#ifndef __UTILS_SEMAPHORE_H__
#define __UTILS_SEMAPHORE_H__

#include <condition_variable>
#include <mutex>

namespace estnet {

/**
 * A Semaphore implementation
 */
class ESTNET_API Semaphore {
private:
    std::mutex _mutex;
    std::condition_variable _condition;
    long _count = 0;
    bool _waiting = false;

public:
    /** @brief decrease semphore counter */
    void notify() {
        std::unique_lock<decltype(_mutex)> lock(_mutex);
        if (_waiting) {
            --_count;
            _condition.notify_all();
        }
    }

    /** @brief wait until semaphore counted to zero */
    void waitForZero(long from) {
        std::unique_lock<decltype(_mutex)> lock(_mutex);
        _count = from;
        _waiting = true;
        while (_count > 0) { // Handle spurious wake-ups.
            _condition.wait(lock);
        }
        _waiting = false;
    }
};

}
 // namespace estnet

#endif
