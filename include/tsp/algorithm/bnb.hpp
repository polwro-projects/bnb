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

#pragma once

#include "math/matrix.hpp"
#include "tsp/algorithm/algorithm.hpp"

namespace tsp::algorithm
{
class BnB : public Algorithm
{
public:
    BnB(math::Matrix<uint32_t> distances);

public:
    Solution Solve(uint32_t start) override;

protected:
    void CalculateSolution(Path path, double bound, double weight);

    double CalculateLowerBound() const;
    double CalculateLowerBound(const std::pair<uint32_t, uint32_t>& positions, uint32_t level) const;

    inline uint32_t GetNearestPathWeight(uint32_t row, uint32_t level = 1) const;

    bool AcceptSolution(const Path& path, double weight);

private:
    Solution solution_;
};
} // namespace tsp::algorithm