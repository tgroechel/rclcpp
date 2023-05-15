// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include "managed_entities_manager.hpp"

namespace rclcpp_lifecycle
{

void
ManagedEntitiesManager::on_activate() const
{
  for (const auto & weak_entity : weak_managed_entities_) {
    auto entity = weak_entity.lock();
    if (entity) {
      entity->on_activate();
    }
  }
}

void
ManagedEntitiesManager::on_deactivate() const
{
  for (const auto & weak_entity : weak_managed_entities_) {
    auto entity = weak_entity.lock();
    if (entity) {
      entity->on_deactivate();
    }
  }
}

void
ManagedEntitiesManager::add_managed_entity(
std::weak_ptr<rclcpp_lifecycle::ManagedEntityInterface> managed_entity)
{
    weak_managed_entities_.push_back(managed_entity);
}

void
ManagedEntitiesManager::add_timer_handle(
std::shared_ptr<rclcpp::TimerBase> timer)
{
    weak_timers_.push_back(timer);
}

}  // namespace rclcpp_lifecycle
