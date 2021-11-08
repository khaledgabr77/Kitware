//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Authors: Guilbert Pierre (Kitware SAS)
//          Laurenson Nick (Kitware SAS)
// Creation date: 2018-03-27
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
//==============================================================================

#include "vtkSpinningSensorKeypointExtractor.h"

#include <vtkObjectFactory.h>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSpinningSensorKeypointExtractor)

//-----------------------------------------------------------------------------
vtkSpinningSensorKeypointExtractor::vtkSpinningSensorKeypointExtractor()
  : Extractor(std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>())
{}

void vtkSpinningSensorKeypointExtractor::PrintSelf(std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "SpinningSensorKeypointExtractor parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->Extractor->Get##param() << std::endl;

  PrintParameter(NeighborWidth)
  PrintParameter(MinDistanceToSensor)

  PrintParameter(PlaneSinAngleThreshold)

  PrintParameter(EdgeSinAngleThreshold)
  PrintParameter(EdgeDepthGapThreshold)
  PrintParameter(EdgeSaliencyThreshold)
  PrintParameter(EdgeIntensityGapThreshold)

  PrintParameter(NbLaserRings)
  PrintParameter(AzimuthalResolution)
}