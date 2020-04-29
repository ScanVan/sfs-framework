/*
 *  sfs-framework
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Charles Papon - charles.papon.90@gmail.com
 *      Copyright (c) 2019-2020 DHLAB, EPFL & HES-SO Valais-Wallis
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// External includes
#include <Eigen/Core>
#include <string>
#include <time.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>
#include <experimental/filesystem>
#include <sstream>
#include <iomanip>

// Namespaces
namespace fs = std::experimental::filesystem;

// Module object
template <typename T> class BlockingQueue{
	std::mutex pushMutex, popMutex, queueMutex;
	std::condition_variable pushCond, popCond;
	uint32_t sizeMax, occupancy;
	std::queue<std::future<T>> queue;
public:
	BlockingQueue(){
		this->sizeMax = -1;
		this->occupancy = 0;
	}
	BlockingQueue(int sizeMax){
		this->sizeMax = sizeMax;
		this->occupancy = 0;
	}

	void push(std::future<T>& e){
		queueMutex.lock();
		queue.push(std::move(e));
		occupancy++;
		popCond.notify_one();
		queueMutex.unlock();

		while(true){
			queueMutex.lock();
			if(occupancy < sizeMax) break;
			queueMutex.unlock();
			std::unique_lock<std::mutex> lk(pushMutex);
			pushCond.wait(lk);
		}

		queueMutex.unlock();
	}

	T pop(){
		while(true){
			queueMutex.lock();
			if(occupancy != 0) break;
			queueMutex.unlock();
			std::unique_lock<std::mutex> lk(popMutex);
			popCond.wait(lk);
		}

		std::future<T> f = std::move(queue.front());
		queueMutex.unlock();

		T e = std::move(f.get());

		queueMutex.lock();
		queue.pop();
		occupancy--;
		pushCond.notify_one();
		queueMutex.unlock();

		return std::move(e);
	}
};

// Module object
template<typename T> std::pair<bool, int> findInVector(
        const std::vector<T> &vecOfElements, const T &element) {
    std::pair<bool, int> result;

    // Find given element in vector
    auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);

    if (it != vecOfElements.end()) {
        result.second = distance(vecOfElements.begin(), it);
        result.first = true;
    } else {
        result.first = false;
        result.second = -1;
    }

    return result;
}

Eigen::Vector3d convertCartesian2Spherical (double x, double y, int width, int height);
void profile(std::string msg);

void exitRetain();
void exitRelease();

double bilinear_sample(double *p, double x, double y, int width);
double bilinear_sample(float *p, double x, double y, int width);

void create_directories( std::string rootPath, std::string modeName );

Eigen::Vector3d compute_intersection(Eigen::Vector3d * p1, Eigen::Vector3d * d1, Eigen::Vector3d * p2, Eigen::Vector3d * d2);

