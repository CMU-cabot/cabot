/*******************************************************************************
 * Copyright (c) 2019, 2022  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

/*
 * Diff Drive
 * 
 * Author: Daisuke Sato <daisukes@cmu.edu>
 */

#include "diff_drive.h"

namespace MotorAdapter
{
DiffDrive::DiffDrive(const double &bias):
  bias_(bias),
  lastTime_(0),
  lastLR_(0,0),
  lastVel_(0,0),
  pose_(0,0,0),
  initialized_(false)
{
}

DiffDrive::~DiffDrive(){

}

void DiffDrive::set(const double &bias) {
  bias_ = bias;
}
    
void DiffDrive::update(const double &left, const double &right, double const &currentTime){
  std::lock_guard<std::mutex> guard(stateMutex_);

  LRdouble currentLR(left, right);

  if (!initialized_){
    lastLR_ = currentLR;
    lastTime_ = currentTime;
    initialized_ = true;
    return;
  }

  LRdouble diffLR = currentLR - lastLR_;
  LRdouble dist;

  dist.r = (diffLR.r - diffLR.l) / bias_;
  dist.l = (diffLR.r + diffLR.l) / 2.0;

  pose_.forward(dist);

  if (currentTime != lastTime_) {
    double diffTime = currentTime - lastTime_;
    lastVel_ = lastVel_ * 0.9 + (dist / diffTime) * 0.1;
    lastLR_ = currentLR;
    lastTime_ = currentTime;
  }
}

Pose& DiffDrive::pose() {
  return pose_;
}

LRdouble& DiffDrive::velocity() {
  return lastVel_;
}

void DiffDrive::y(const double &y) {
  pose_.y = y;
}
}  // namespace MotorAdapter
