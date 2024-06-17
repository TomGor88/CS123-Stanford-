/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/
#pragma once
#include <kinematics.h>
#include <utils.h>
void test_ik1()
{
  const KinematicsConfig pupper_leg_config = KinematicsConfig{-0.035, 0.08, 0.11};
  BLA::Matrix<3> actuator_angles1(PI / 2, PI / 2, PI / 2);
  BLA::Matrix<3> test_point(-0.1, 0.07, 0);
  BLA::Matrix<3> angles = inverse_kinematics(test_point, pupper_leg_config, actuator_angles1);
  BLA::Matrix<3> res = (forward_kinematics(angles, pupper_leg_config));
  assert_close(res(0), -0.10, 0.01);
  assert_close(res(1), 0.07, 0.01);
  assert_close(res(2), 0.0, 0.01);
}

void test_inv_kinematics()
{
  test_ik1();
}