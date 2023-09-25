/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * This source code is licensed under the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#pragma once
#include <chrono>
#include <functional>
#include "FLIP.h"

namespace split_rendering {

class FlipScreenshotComparison {
 public:
  static void compare(
      std::string& referenceFilename,
      std::string& currentFilename,
      std::string& destinationDirectory) {
    // Most of the code below was copied from the FLIP main executable.
    struct {
      float ppd = 0; // If ppd==0.0, then it will be computed from the parameters below.
      float monitorDistance = 0.7f; // Unit: meters.
      float monitorWidth = 0.7f; // Unit: meters.
      float monitorResolutionX = 3840.0f; // Unit: pixels.
    } gFLIPOptions;

    FLIP::filename referenceFileName = referenceFilename;
    FLIP::filename testFileName = currentFilename;

    FLIP::image<FLIP::color3> magmaMap(FLIP::MapMagma, 256);

    const auto& calculatePPD =
        [](const float dist, const float resolutionX, const float monitorWidth) {
          return dist * (resolutionX / monitorWidth) * (float(FLIP::PI) / 180.0f);
        };

    gFLIPOptions.ppd = calculatePPD(
        gFLIPOptions.monitorDistance, gFLIPOptions.monitorResolutionX, gFLIPOptions.monitorWidth);

    FLIP::image<FLIP::color3> referenceImage(referenceFileName.toString());

    float startExposure = 0.0f, stopExposure = 0.0f;
    size_t numExposures = 0;
    float exposureStepSize = 0.0f;
    bool startExp = false;
    bool stopExp = false;

    FLIP::filename flipFileName("tmp.png");
    FLIP::filename histogramFileName("tmp.py");
    FLIP::filename exposureFileName("tmp.png");
    FLIP::filename csvFileName("tmp.csv");

    FLIP::image<FLIP::color3> originalReferenceImage(
        referenceImage.getWidth(), referenceImage.getHeight());

    flipFileName.setName(
        referenceFileName.getName() + "." + testFileName.getName() + "." +
        std::to_string(int(std::round(gFLIPOptions.ppd))) + "ppd");

    flipFileName.setName(flipFileName.getName() + ".ldr");
    histogramFileName.setName("weighted_histogram." + flipFileName.getName());
    csvFileName.setName("csv." + flipFileName.getName());
    exposureFileName.setName("exposure_map." + flipFileName.getName());
    flipFileName.setName("flip." + flipFileName.getName());

    FLIP::image<FLIP::color3> testImage(testFileName.toString());

    FLIP::image<FLIP::color3> viridisMap(FLIP::MapViridis, 256);
    FLIP::image<float> errorMapFLIP(referenceImage.getWidth(), referenceImage.getHeight());

    auto t0 = std::chrono::high_resolution_clock::now();
    auto t = t0;
    errorMapFLIP.FLIP(referenceImage, testImage, gFLIPOptions.ppd);
    t = std::chrono::high_resolution_clock::now();
    FLIP::image<FLIP::color3> pngResult(referenceImage.getWidth(), referenceImage.getHeight());
    pngResult.copyFloat2Color3(errorMapFLIP);
    pngResult.colorMap(errorMapFLIP, magmaMap);
    pngResult.pngSave(destinationDirectory + "/" + flipFileName.toString());

    pooling<float> pooledValues;
    for (int y = 0; y < errorMapFLIP.getHeight(); y++) {
      for (int x = 0; x < errorMapFLIP.getWidth(); x++) {
        pooledValues.update(x, y, errorMapFLIP.get(x, y));
      }
    }

    bool optionLog = false;
    bool optionExcludeValues = false;
    float yMax = 1.0f;

    pooledValues.save(
        destinationDirectory + "/" + histogramFileName.toString(),
        errorMapFLIP.getWidth(),
        errorMapFLIP.getHeight(),
        optionLog,
        referenceFileName.toString(),
        testFileName.toString(),
        !optionExcludeValues,
        yMax);

    std::fstream csv;
    csv.open(destinationDirectory + "/" + csvFileName.toString(), std::ios::app);
    if (csv.is_open()) {
      csv.seekp(0, std::ios_base::end);

      if (csv.tellp() <= 0)
        csv << "\"Reference\",\"Test\",\"Mean\",\"Weighted median\",\"1st weighted quartile\",\"3rd weighted quartile\",\"Min\",\"Max\",\"Evaluation time\"\n";

#define FIXED_DECIMAL_DIGITS(x, d) std::fixed << std::setprecision(d) << (x)

      csv << "\"" << referenceFileName.toString() << "\",";
      csv << "\"" << testFileName.toString() << "\",";
      csv << "\"" << FIXED_DECIMAL_DIGITS(pooledValues.getMean(), 6) << "\",";
      csv << "\"" << FIXED_DECIMAL_DIGITS(pooledValues.getPercentile(0.5f, true), 6) << "\",";
      csv << "\"" << FIXED_DECIMAL_DIGITS(pooledValues.getPercentile(0.25f, true), 6) << "\",";
      csv << "\"" << FIXED_DECIMAL_DIGITS(pooledValues.getPercentile(0.75f, true), 6) << "\",";
      csv << "\"" << FIXED_DECIMAL_DIGITS(pooledValues.getMinValue(), 6) << "\",";
      csv << "\"" << FIXED_DECIMAL_DIGITS(pooledValues.getMaxValue(), 6) << "\",";
      csv << "\""
          << FIXED_DECIMAL_DIGITS(
                 std::chrono::duration_cast<std::chrono::microseconds>(t - t0).count() / 1000000.0f,
                 4)
          << "\"\n";

      csv.close();
    } else {
      std::cout << "\nError: Could not write csv file " << csvFileName.toString() << "\n";
    }
  }
};

} // namespace split_rendering
