/**
 * @copyright Copyright (c) 2019 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential!
 *
 * @brief marker class
 */

#define STD_APP_MARKER_POSITION_ID (10000)
#define STD_APP_MARKER_POSITION_NAME "Marker position"

#include <fstream>
#include <sstream>

// protobuf to json
#include <google/protobuf/util/json_util.h>

#include "vs_c_marker.h"
//#include "vs_json.h"
//#include "vs_jsonMarkerObject.h"
#include "vs_messaging.h"
// protofiles
#include "Marker.pb.h"
//#include "Program.pb.h"

using namespace VMS::VisionSystem;  // protofiles

static json j;
static std::ostringstream sConfigFile;

// start nmarker detection - load modul for marker detection and wait for image
bool MarkerClass::processMarkerStart(std::string sMessageBuffer, std::string& result)
{
	MarkerPosition markerPosition;
	// can not extract/parse message
	if (!markerPosition.ParseFromString(sMessageBuffer))
	{
		return false;
	}

	int nVisionSystemNumber = markerPosition.vision_system_id();

	// deleted legacy code. see git for details

	return true;
}

// en  marker detection - load modul for marker detection and wait for image

bool MarkerClass::processMarkerEnd(std::string sMessageBuffer, std::string& result)
{
	MarkerPosition markerPosition;
	// can not extract/parse message
	if (!markerPosition.ParseFromString(sMessageBuffer))
	{
		return false;
	}

	// deleted legacy code. see git for details

	return true;
}

// marker detection - get marker value from db
// send marker settings to PS
bool MarkerClass::processMarkerGet(std::string sMessageBuffer, std::string& result)
{
	MarkerPosition markerPosition;
	// can not extract/parse message
	if (!markerPosition.ParseFromString(sMessageBuffer))
	{
		return false;
	}

	// deleted legacy code. see git for details

	return true;
}

// marker detection - set marker value from PS to db
bool MarkerClass::processMarkerSet(std::string sMessageBuffer, std::string& result)
{
	MarkerPosition markerPosition;
	// can not extract/parse message
	if (!markerPosition.ParseFromString(sMessageBuffer))
	{
		return false;
	}

	// deleted legacy code. see git for details

	return true;
}
