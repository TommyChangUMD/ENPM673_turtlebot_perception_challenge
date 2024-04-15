/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
  Changes
  2024-04-13: Tommy Chang: Added trajectory parser code taken from gazebo/gazebo/physics/Actor.cc 
*/

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // check trajectory 
      if (! _sdf->HasElement("trajectory"))
        {
          std::cerr << "Animated_box plugin: No trajectory defined." << std::endl;
          return;
        }
      
      // Load trajectory containg waypoints
      common::PoseAnimation *animPtr = NULL;
      animPtr = this->LoadTrajectory(_sdf->GetElement("trajectory"));

      // check trajectory loading result
      if (animPtr == NULL)
        {
          std::cerr << "Animated_box plugin: No waypoints defined." << std::endl;
          return;
        }

      // set the animation
      gazebo::common::PoseAnimationPtr anim (animPtr);
      _parent->SetAnimation (anim);

      std::cerr << "Animated_box plugin is loaded" << std::endl;
    }

    private:
    // member variables:
    physics::ModelPtr    model; // Pointer to the model
    event::ConnectionPtr updateConnection; // Pointer to the update event connection

    // member funcitions:
    common::PoseAnimation* LoadTrajectory(sdf::ElementPtr _sdf);
  };

  common::PoseAnimation *AnimatedBox::LoadTrajectory(sdf::ElementPtr trajSdf)
  {
    // Waypoints
    if (! trajSdf->HasElement("waypoint"))
      return NULL;
      
    // Fill a map with waypoints time and pose
    std::map<double, ignition::math::Pose3d> points;
    sdf::ElementPtr wayptSdf = trajSdf->GetElement("waypoint");
    while (wayptSdf)
      {
        points[wayptSdf->Get<double>("time")] =
          wayptSdf->Get<ignition::math::Pose3d>("pose");
        wayptSdf = wayptSdf->GetNextElement("waypoint");
      }

    // Get total trajectory duration (last waypoint's time)
    auto last = points.rbegin();

    // Create animation 
    common::PoseAnimation *anim = new common::PoseAnimation("my_name", last->first, true);

    // Create a keyframe for each point
    for (auto pIter = points.begin(); pIter != points.end(); ++pIter)
      {
        common::PoseKeyFrame *key;
        // Force first point always to start at 0s
        if (pIter == points.begin() &&
            !ignition::math::equal(pIter->first, 0.0))
          key = anim->CreateKeyFrame(0.0);
        else
          key = anim->CreateKeyFrame(pIter->first);

        key->Translation(pIter->second.Pos());
        key->Rotation(pIter->second.Rot());
      }

    return anim;
  }
  
  
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}
