/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef OPEN_SPACE_PLANNER_REEDS_SHEPP_PATH_H
#define OPEN_SPACE_PLANNER_REEDS_SHEPP_PATH_H

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

//#include <omp.h>

#include "HybridAStarNode.h"

using namespace hybrid_a_star;

namespace reeds_shepp
{
  struct ReedsSheppPath
  {
    std::vector<float> segs_lengths;
    std::vector<char> segs_types;
    float total_length = 0.0;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> phi;
    // true for driving forward and false for driving backward
    std::vector<bool> gear;
  };

  struct RSPParam
  {
    bool flag = false;
    float t = 0.0;
    float u = 0.0;
    float v = 0.0;
  };

  class ReedsShepp
  {
  public:
    ReedsShepp(const float &max_kappa_, bool dubins_only_);
    virtual ~ReedsShepp() = default;
    // Pick the shortest path from all possible combination of movement primitives
    // by Reed Shepp
    bool ShortestRSP(const std::shared_ptr<HybridAStarNode> start_node,
                     const std::shared_ptr<HybridAStarNode> end_node,
                     std::shared_ptr<ReedsSheppPath> optimal_path);

  protected:
    // Generate all possible combination of movement primitives by Reed Shepp and
    // interpolate them
    bool GenerateRSPs(const std::shared_ptr<HybridAStarNode> start_node,
                      const std::shared_ptr<HybridAStarNode> end_node,
                      std::vector<ReedsSheppPath> *all_possible_paths);
    // Set the general profile of the movement primitives
    bool GenerateRSP(const std::shared_ptr<HybridAStarNode> start_node,
                     const std::shared_ptr<HybridAStarNode> end_node,
                     std::vector<ReedsSheppPath> *all_possible_paths);
    // Set the general profile of the movement primitives, parallel implementation
    bool GenerateRSPPar(const std::shared_ptr<HybridAStarNode> start_node,
                        const std::shared_ptr<HybridAStarNode> end_node,
                        std::vector<ReedsSheppPath> *all_possible_paths);
    // Set local exact configurations profile of each movement primitive
    bool GenerateLocalConfigurations(const std::shared_ptr<HybridAStarNode> start_node,
                                     const std::shared_ptr<HybridAStarNode> end_node,
                                     ReedsSheppPath *shortest_path);
    // Interpolation usde in GenetateLocalConfiguration
    void Interpolation(const int index, const float pd, const char m,
                       const float ox, const float oy, const float ophi,
                       std::vector<float> *px, std::vector<float> *py,
                       std::vector<float> *pphi, std::vector<bool> *pgear);
    // motion primitives combination setup function
    bool SetRSP(const int size, const float *lengths, const char *types,
                std::vector<ReedsSheppPath> *all_possible_paths);
    // setRSP parallel version
    bool SetRSPPar(const int size, const float *lengths,
                   const std::string &types,
                   std::vector<ReedsSheppPath> *all_possible_paths, const int idx);
    /* Six different combination of motion primitive in Reed Shepp path used in GenerateRSP() */
    // SCS: 2 combinations
    bool SCS(const float x, const float y, const float phi,
             std::vector<ReedsSheppPath> *all_possible_paths);
    // CSC: 8 combinations
    bool CSC(const float x, const float y, const float phi,
             std::vector<ReedsSheppPath> *all_possible_paths);
    // CCC: 8 combinations
    bool CCC(const float x, const float y, const float phi,
             std::vector<ReedsSheppPath> *all_possible_paths);
    // CCCC: 8 combinations
    bool CCCC(const float x, const float y, const float phi,
              std::vector<ReedsSheppPath> *all_possible_paths);
    // CCSC: 16 combinations
    bool CCSC(const float x, const float y, const float phi,
              std::vector<ReedsSheppPath> *all_possible_paths);
    // CCSCC: 4 combinations
    bool CCSCC(const float x, const float y, const float phi,
               std::vector<ReedsSheppPath> *all_possible_paths);
    // different options for different combination of motion primitives
    void LSL(const float x, const float y, const float phi, RSPParam *param);
    void LSR(const float x, const float y, const float phi, RSPParam *param);
    void LRL(const float x, const float y, const float phi, RSPParam *param);
    void SLS(const float x, const float y, const float phi, RSPParam *param);
    void LRLRn(const float x, const float y, const float phi, RSPParam *param);
    void LRLRp(const float x, const float y, const float phi, RSPParam *param);
    void LRSR(const float x, const float y, const float phi, RSPParam *param);
    void LRSL(const float x, const float y, const float phi, RSPParam *param);
    void LRSLR(const float x, const float y, const float phi, RSPParam *param);
    std::pair<float, float> calc_tau_omega(const float u, const float v,
                                           const float xi, const float eta,
                                           const float phi);

    std::pair<float, float> Cartesian2Polar(float x, float y);

  protected:
    float m_max_kappa;
    float m_step_size = 0.05;
    bool m_dubins_only = true;
  };
} // namespace reeds_shepp
#endif
