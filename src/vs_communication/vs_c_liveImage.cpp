/**
 * @copyright Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief liveimage of camera or visionsystem
 *
 */
#include "vs_c_liveImage.h"

// protofiles
#include "LiveImage.pb.h"

using namespace VMS::VisionSystem;  // protofiles

// switch on live image from camera
bool LiveImage::processLiveImageCamera(std::string sMessageBuffer, std::string& result) { return true; }

// switch on live image from vision system
bool LiveImage::processLiveImageVisionSystem(std::string sMessageBuffer, std::string& result) { return true; }
