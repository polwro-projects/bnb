/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "tsp/algorithm/bnb.hpp"

#include <algorithm>
#include <stdexcept>

namespace tsp::algorithm
{
BnB::BnB(math::Matrix<uint32_t> distances) : Algorithm{ distances }
{
}

Algorithm::Solution BnB::Solve(uint32_t start)
{
    CalculateSolution({ start }, CalculateLowerBound(), 0);
    return solution_;
}

void BnB::CalculateSolution(Path path, double bound, double weight)
{
    for (uint32_t position{}; position < distances_.Columns(); ++position)
    {
        // Don't do anything, if the position is already in the list
        if (std::find(path.begin(), path.end(), position) != path.end())
        {
            continue;
        }

        const auto level = path.size();
        const auto currentBound = bound - CalculateLowerBound({ path[level - 1], position }, level);
        const auto currentWeight = weight + distances_(path[level - 1], position);
        if (currentBound + currentWeight < solution_.weight)
        {
            auto currentPath = path;
            currentPath.push_back(position);

            if (AcceptSolution(currentPath, currentWeight))
                continue;

            CalculateSolution(currentPath, currentBound, currentWeight);
        }
    }
}

bool BnB::AcceptSolution(const Path& path, double weight)
{
    const uint32_t level = path.size();
    if (level == distances_.Columns())
    {
        weight += distances_(path.at(level - 1), path.at(0));
        if (weight > solution_.weight)
        {
            return false;
        }

        solution_ = { path, weight };
        return true;
    }

    return false;
}

double BnB::CalculateLowerBound() const
{
    uint32_t result{};
    for (uint32_t position{ 0 }; position < distances_.Columns(); ++position)
    {
        result += GetNearestPathWeight(position);
        result += GetNearestPathWeight(position, 2);
    }
    return result / 2;
}

inline double BnB::CalculateLowerBound(const std::pair<uint32_t, uint32_t>& positions, uint32_t level) const
{
    const uint32_t result =
        GetNearestPathWeight(positions.second) + GetNearestPathWeight(positions.first, (level == 1) ? 1 : 2);
    return result / 2;
}

inline uint32_t BnB::GetNearestPathWeight(uint32_t position, uint32_t level) const
{
    auto adjacencyList = distances_.GetRow(position);
    std::sort(adjacencyList.begin(), adjacencyList.end());
    return adjacencyList.at(level);
}
} // namespace tsp::algorithm