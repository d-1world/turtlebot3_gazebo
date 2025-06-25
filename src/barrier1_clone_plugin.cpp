// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Hyungyu Kim

// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0




// #include "turtlebot3_gazebo/barrier1_plugin.hpp"

// #include <iostream>

// namespace gazebo
// {
// Barrier1Plugin::Barrier1Plugin()
// : move_cycle(0.5),
//   going_to_pose2(true),
//   is_waiting(false),                        // [추가]
//   wait_duration(1.0)                        // [추가] 멈춤 시간 2초
// {
// }

// void Barrier1Plugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
// {
//   this->model = _model;
//   this->world = _model->GetWorld();

//   std::cout << "[Barrier1Plugin] Loaded for model: " << this->model->GetName() << std::endl;

//   this->pose_1 = ignition::math::Pose3d(
//     ignition::math::Vector3d(0.010181, 0.760888, 0.10),
//     ignition::math::Quaterniond(0.0, 0.0, 1.58756));
//   this->pose_2 = ignition::math::Pose3d(
//     ignition::math::Vector3d(-0.640166, 0.749985, 0.10),
//     ignition::math::Quaterniond(-0.000001, 0.000004, 1.587558));

//   this->last_time = this->world->SimTime();
//   this->wait_start_time = this->last_time;  // [추가]

//   this->model->SetGravityMode(false);

//   this->update_connection = event::Events::ConnectWorldUpdateBegin(
//     std::bind(&Barrier1Plugin::OnUpdate, this));
// }

// void Barrier1Plugin::OnUpdate()
// {
//   common::Time current_time = this->world->SimTime();

//   auto link = this->model->GetLink("link");
//   if (!link) return;

//   ignition::math::Vector3d current_pos = this->model->WorldPose().Pos();
//   ignition::math::Vector3d target_pos = going_to_pose2 ? pose_2.Pos() : pose_1.Pos();
//   ignition::math::Vector3d move_dir = target_pos - current_pos;
//   move_dir.Z() = 0.0;

//   double speed = 0.1;  // m/s

//   if (is_waiting) {
//     // [추가] 대기 중일 경우 일정 시간 기다림
//     if ((current_time - wait_start_time).Double() >= wait_duration) {
//       is_waiting = false;
//     } else {
//       link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
//       return;
//     }
//   }

//   if (move_dir.Length() < 0.01) {
//     // [수정] 도착 시 대기 시작
//     going_to_pose2 = !going_to_pose2;
//     is_waiting = true;
//     wait_start_time = current_time;
//     link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
//     return;
//   }

//   move_dir.Normalize();
//   link->SetLinearVel(move_dir * speed);

//   this->last_time = current_time;
// }
// }  // namespace gazebo



#include "turtlebot3_gazebo/barrier1_clone_plugin.hpp"
#include <iostream>

namespace gazebo
{
Barrier1ClonePlugin::Barrier1ClonePlugin()
: pose_index(0), is_waiting(false), wait_duration(1.0) {}

void Barrier1ClonePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->world = _model->GetWorld();

  std::cout << "[Barrier1ClonePlugin] Loaded for model: " << this->model->GetName() << std::endl;

  // 이동할 위치들 (기역자 경로)
  ignition::math::Pose3d pose_1(
    ignition::math::Vector3d(0.07968, 1.705974, 0.01),
    ignition::math::Quaterniond(0,000002, 0.000004, -0.010490));
  ignition::math::Pose3d pose_2(
    ignition::math::Vector3d(0.515451, 1.701419, 0.01),
    ignition::math::Quaterniond(0, -0.000002, -0.010436));
  ignition::math::Pose3d pose_3(
    ignition::math::Vector3d(0.506529, 0.846405, 0.01),
    ignition::math::Quaterniond(0, 0.000002, -0.010436));

  this->poses = {pose_1, pose_2, pose_3, pose_2, pose_1};
  this->pose_index = 0;

  this->last_time = this->world->SimTime();
  this->wait_start_time = this->last_time;

  // 중력 영향 제거
  this->model->SetGravityMode(false);

  // 업데이트 함수 등록
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&Barrier1ClonePlugin::OnUpdate, this));
}

void Barrier1ClonePlugin::OnUpdate()
{
  common::Time current_time = this->world->SimTime();
  auto link = this->model->GetLink("link");
  if (!link) return;

  ignition::math::Vector3d current_pos = this->model->WorldPose().Pos();
  ignition::math::Vector3d target_pos = poses[pose_index].Pos();
  ignition::math::Vector3d move_dir = target_pos - current_pos;
  move_dir.Z() = 0.0;

  double speed = 0.1;  // m/s

  if (is_waiting) {
    if ((current_time - wait_start_time).Double() >= wait_duration) {
      is_waiting = false;
    } else {
      link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
      return;
    }
  }

  if (move_dir.Length() < 0.01) {
    pose_index = (pose_index + 1) % poses.size();
    is_waiting = true;
    wait_start_time = current_time;
    link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
    return;
  }

  move_dir.Normalize();
  link->SetLinearVel(move_dir * speed);

  last_time = current_time;
}
}  // namespace gazebo
