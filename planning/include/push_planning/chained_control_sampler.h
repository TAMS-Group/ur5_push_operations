/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Lars Henning Kayser
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Lars Henning Kayser */

#pragma once

#include <ompl/base/goals/GoalState.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <push_planning/conversions.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace push_planning {
  class ChainedControlSampler : public oc::DirectedControlSampler {
    private:

      oc::ControlSamplerPtr cs_;
      unsigned int numControlSamples_;
      const oc::Control* previous_init_control_;

      std::random_device rd;
      std::mt19937 gen{rd()};
      std::normal_distribution<double> dist_{0.0, 0.5};
    public:
      ChainedControlSampler(const oc::SpaceInformation *si, unsigned int k, const oc::Control* last_control)
        : oc::DirectedControlSampler(si),
        cs_(si->allocControlSampler()),
        numControlSamples_(k),
        previous_init_control_(last_control)
    {
      }

      ~ChainedControlSampler() = default;

      unsigned int getNumControlSamples() const
      {
        return numControlSamples_;
      }

      void setNumControlSamples(unsigned int numSamples)
      {
        numControlSamples_ = numSamples;
      }

      unsigned int sampleTo(oc::Control *control, const ob::State *source,
          ob::State *dest)
      {
        return getBestControl(control, source, dest, nullptr);
      }

      unsigned int sampleTo(oc::Control *control, const oc::Control *previous,
          const ob::State *source, ob::State *dest)
      {
        return getBestControl(control, source, dest, previous);
      }

      /*
       * Sample next control by varying the approach point by +-10 percent randomly
       */
      unsigned int getBestControl(oc::Control *control, const ob::State *source,
          ob::State *dest, const oc::Control *previous)
      {
        cs_->sample(control, source);

        double previous_approach = 0.0;
        // Sample the first control
        if (previous != nullptr) {
          previous_approach = previous->as<oc::RealVectorControlSpace::ControlType>()->values[0];
          if (previous_approach > 0.0)
            control->as<oc::RealVectorControlSpace::ControlType>()->values[0] = std::fmod(dist_(gen) * 0.1 + previous_approach, 1.0);
        }

        const unsigned int minDuration = si_->getMinControlDuration();
        const unsigned int maxDuration = si_->getMaxControlDuration();

        unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);
        // Propagate the first control, and find how far it is from the target state
        ob::State *bestState = si_->allocState();
        steps = si_->propagateWhileValid(source, control, steps, bestState);

        if (numControlSamples_ > 1)
        {
          oc::Control *tempControl = si_->allocControl();
          ob::State *tempState = si_->allocState();
          double bestDistance = si_->distance(bestState, dest);

          // Sample k-1 more controls, and save the control that gets closest to target
          for (unsigned int i = 1; i < numControlSamples_; ++i)
          {
            cs_->sample(tempControl, source);
            if (previous_approach > 0.0)
              tempControl->as<oc::RealVectorControlSpace::ControlType>()->values[0] = std::fmod(dist_(gen) * 0.1 + previous_approach, 1.0);
            unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
            sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
            double tempDistance = si_->distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
              si_->copyState(bestState, tempState);
              si_->copyControl(control, tempControl);
              bestDistance = tempDistance;
              steps = sampleSteps;
            }
          }

          si_->freeState(tempState);
          si_->freeControl(tempControl);
        }

        si_->copyState(dest, bestState);
        si_->freeState(bestState);

        return steps;
      }
  };
}
