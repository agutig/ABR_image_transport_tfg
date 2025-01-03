// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FFMPEG_IMAGE_TRANSPORT__TDIFF_HPP_
#define FFMPEG_IMAGE_TRANSPORT__TDIFF_HPP_

#include <iostream>

namespace abr_ffmpeg_image_transport
{
class TDiff
{
public:
  friend std::ostream & operator<<(std::ostream & os, const TDiff & td);
  inline void update(double dt)
  {
    duration_ += dt;
    cnt_++;
  }
  inline void reset()
  {
    duration_ = 0;
    cnt_ = 0;
  }

private:
  int64_t cnt_{0};
  double duration_{0};
};
std::ostream & operator<<(std::ostream & os, const TDiff & td);

}  // namespace abr_ffmpeg_image_transport

#endif  // FFMPEG_IMAGE_TRANSPORT__TDIFF_HPP_
