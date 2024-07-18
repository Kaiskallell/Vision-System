/**
 * @copyright Copyright (c) 2022 Gerhard Schubert GmbH - All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#include "yoloDLCPipeline.h"

#include <iostream>
#include <opencv2/opencv.hpp>

#include "inferDLC.h"
#include "miscellaneous.h"
#include "projectPaths.h"
#include "trtLogging.h"
#include "yolo.h"

// only for debugging
void drawAndShowDetections(cv::Mat& debugImg,
                           const YoloDLCResults& keyPointsAndClasses,
                           const std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses,
                           const std::vector<std::vector<float>>& keyPointConfs)
{
	for (int i = 0; i < rectsAndClasses->size(); ++i)
	{
		if (keyPointsAndClasses.m_keyPoints.empty() || keyPointsAndClasses.m_keyPoints.size() - 1 < i)
		{
			continue;
		};

		for (int j = 0; j < keyPointsAndClasses.m_keyPoints.at(i).size(); ++j)
		{
			cv::circle(debugImg, keyPointsAndClasses.m_keyPoints.at(i).at(j), 2, cv::Scalar(0, 255, 255), 2);
			std::string confStr = std::to_string(keyPointConfs[i][j]);
			std::string finalConfStr = confStr.substr(0, confStr.find(".") + 3);
			cv::putText(debugImg,
			            std::to_string(j) /*+ " " + finalConfStr*/,
			            keyPointsAndClasses.m_keyPoints.at(i).at(j),
			            cv::FONT_HERSHEY_PLAIN,
			            1.2,
			            cv::Scalar(0xFF, 0xFF, 0xFF),
			            2);
		}
	}
	misc::showImage("detections", debugImg);
	misc::waitKey1ms();
}

class YoloDLCPipeline::Impl
{
  public:
	Impl() = default;
	~Impl() = default;
	std::vector<std::unique_ptr<dlc::InferDLC>> m_dlcs;
	std::unique_ptr<yolo::Yolo> m_yolo;
};

YoloDLCPipeline::YoloDLCPipeline(const fs::path& networksPath)
{
	m_impl = std::make_unique<Impl>();

	nlohmann::json jsonFile = utils::loadJsonFile(networksPath);
	try
	{
		for (auto it = jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("classToDLCIndexMapping").begin();
				it != jsonFile.at("appConfigs").at("yolo").at("detectionConfigs").at("classToDLCIndexMapping").end();
				++it)
		{
			m_classToDlcIdxMapping[std::stoi(it.key())] = it.value();
		}

		for (int dlcIdx = 0; dlcIdx < jsonFile.at("appConfigs").at("dlcs").size(); ++dlcIdx)
		{
			m_impl->m_dlcs.emplace_back(std::make_unique<dlc::InferDLC>(networksPath, dlcIdx));
			if(!m_impl->m_dlcs.at(dlcIdx)->m_dlcConfig.configured)
			{
				return;
			}
		}
	}
	catch (const std::exception& e)
	{
		m_kLogger->error("Could not read parameters from file: " + networksPath.string());
		return;
	}
	m_impl->m_yolo = std::make_unique<yolo::Yolo>(networksPath);
	if(m_impl->m_yolo->m_config.configured)
	{
		m_moduleConfigured = true;
	}
}

YoloDLCPipeline::~YoloDLCPipeline() { m_kLogger->debug("YoloDLCPipeline decstuctor"); }

YoloDLCResults YoloDLCPipeline::run(cv::Mat& src, const size_t nMaxResults, const int desiredClass)
{
	cv::Mat debugImg = src.clone();
	std::shared_ptr<yolo::RectsAndClasses> rectsAndClasses;
	rectsAndClasses = m_impl->m_yolo->run(src, debugImg);
	// m_kLogger->debug("Yolo detcted {} objects", rectsAndClasses->size());

	// crop src with the detections from yolo and pass it to dlc
	YoloDLCResults keyPointsAndClasses;
	std::vector<std::vector<float>> keypointConfs;
	for (size_t i = 0; i < rectsAndClasses->size(); ++i)
	{
		if (desiredClass != rectsAndClasses->at(i).second
		    && desiredClass != YoloDLC::kAllClasses)  // if desiredClass == -1 we return all detections
		{
			continue;  // we are only interested in detections which are the desired class
		}

		size_t dlcIdx = 0;
		try
		{
			dlcIdx = m_classToDlcIdxMapping.at(rectsAndClasses->at(i).second);
		}
		catch (const std::exception& e)
		{
			m_kLogger->error(e.what());
			throw std::runtime_error("m_classToDlcIdxMapping does not contain requested key: "
			                         + std::to_string(rectsAndClasses->at(i).second));
		}

		cv::RotatedRect rotRect = rectsAndClasses->at(i).first;

		dlc::PointsAndConfs pointsAndConfs = m_impl->m_dlcs.at(dlcIdx)->run(src, rotRect);
		if (pointsAndConfs.keypoints.empty())
		{
			m_kLogger->error("DLC not able to find keypoints for detection (rect)");
			continue;
		}

		keyPointsAndClasses.m_keyPoints.push_back(pointsAndConfs.keypoints);
		keyPointsAndClasses.m_classIds.push_back(rectsAndClasses->at(i).second);
		keyPointsAndClasses.m_rects.push_back(rectsAndClasses->at(i).first.boundingRect());
		keypointConfs.push_back(pointsAndConfs.confs);

		if (keyPointsAndClasses.m_keyPoints.size() >= nMaxResults)
		{
			break;
		}
	}

	drawAndShowDetections(debugImg, keyPointsAndClasses, rectsAndClasses, keypointConfs);

	return keyPointsAndClasses;
}

namespace YoloDLC
{
std::shared_ptr<YoloDLCPipeline> setupYoloDlc(const fs::path& networksConfigPath,
                                              const std::shared_ptr<spdlog::logger> kLogger)
{
	std::shared_ptr<YoloDLCPipeline> yoloDlcPipeline;
	try
	{
		yoloDlcPipeline = std::make_shared<YoloDLCPipeline>(
		    networksConfigPath);  // first object detection with Yolo and then keypoints detection with DeepLabCut
		yoloDlcPipeline->m_moduleConfigured = true;
	}
	catch (const std::exception& e)
	{
		yoloDlcPipeline->m_moduleConfigured = false;
		kLogger->error("Cannot create YoloDLCPipeline");
		kLogger->error(e.what());
		exit(-1);
	}
	return yoloDlcPipeline;
}
}  // namespace YoloDLC
