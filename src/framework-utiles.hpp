/*
 *  sfs-framework
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Charles Papon - charles.papon.90@gmail.com
 *      Copyright (c) 2019 DHLAB, EPFL & HES-SO Valais-Wallis
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

#include <Eigen/Core>
#include <string>
#include <time.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>

Eigen::Vector3d convertCartesian2Spherical (double x, double y, int width, int height);
void profile(std::string msg);

//ThreadPool threadPool(8);
template <typename T> class BlockingQueue{
	std::mutex pushMutex, popMutex, queueMutex;
	std::condition_variable pushCond, popCond;
	uint32_t sizeMax;
	std::queue<std::future<T>> queue;
public:
	BlockingQueue(){
		this->sizeMax = -1;
	}
	BlockingQueue(int sizeMax){
		this->sizeMax = sizeMax;
	}

	void push(std::future<T>& e){
		std::unique_lock<std::mutex> lk(pushMutex);
		pushCond.wait(lk, [this]{return queue.size() != sizeMax;});
		std::lock_guard<std::mutex> l(queueMutex);
		queue.push(std::move(e));
		popCond.notify_one();
	}

	T pop(){
		std::unique_lock<std::mutex> lk(popMutex);
		popCond.wait(lk, [this]{return !queue.empty();});
		std::lock_guard<std::mutex> l(queueMutex);
		T e = std::move(queue.front().get());
		queue.pop();
		pushCond.notify_one();
		return std::move(e);
	}
};
