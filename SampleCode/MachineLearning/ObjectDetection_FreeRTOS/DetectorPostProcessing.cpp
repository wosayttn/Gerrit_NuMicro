/*
 * Copyright (c) 2022 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
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
 */
#include "DetectorPostProcessing.hpp"
#include "PlatformMath.hpp"

#include <cmath>

namespace arm
{
namespace app
{
namespace object_detection
{

DetectorPostprocessing::DetectorPostprocessing(
    const float threshold,
    const float nms,
    int numClasses,
    int topN)
    :   m_threshold(threshold),
        m_nms(nms),
        m_numClasses(numClasses),
        m_topN(topN)
{}

void DetectorPostprocessing::RunPostProcessing(
    uint32_t imgNetRows,
    uint32_t imgNetCols,
    uint32_t imgSrcRows,
    uint32_t imgSrcCols,
    TfLiteTensor *modelOutput0,
    TfLiteTensor *modelOutput1,
    std::vector<DetectionResult> &resultsOut    /* init postprocessing */
)
{
#if defined (__ICCARM__)
    Network net;
    net.inputWidth = static_cast<int>(imgNetCols);
    net.inputHeight = static_cast<int>(imgNetRows);
    net.numClasses = m_numClasses;


    Branch branches0;
    Branch branches1;
    branches0.resolution = modelOutput0->dims->data[1];
    branches0.numBox = 3;
    branches0.anchor = anchor1;
    branches0.modelOutput = modelOutput0->data.int8;
    branches0.scale = ((TfLiteAffineQuantization *)(modelOutput0->quantization.params))->scale->data[0];
    branches0.zeroPoint = ((TfLiteAffineQuantization *)(modelOutput0->quantization.params))->zero_point->data[0];
    branches0.size = modelOutput0->bytes;

    net.branches.push_back(branches0);

    branches1.resolution = modelOutput1->dims->data[1];
    branches1.numBox = 3;
    branches1.anchor = anchor2;
    branches1.modelOutput = modelOutput1->data.int8;
    branches1.scale = ((TfLiteAffineQuantization *)(modelOutput1->quantization.params))->scale->data[0];
    branches1.zeroPoint = ((TfLiteAffineQuantization *)(modelOutput1->quantization.params))->zero_point->data[0];
    branches1.size = modelOutput1->bytes;

    net.branches.push_back(branches1);
    net.topN = m_topN;
#else
    Network net
    {
        .inputWidth = static_cast<int>(imgNetCols),
        .inputHeight = static_cast<int>(imgNetRows),
        .numClasses = m_numClasses,
        .branches = {
            Branch {
                .resolution = modelOutput0->dims->data[1],
                .numBox = 3,
                .anchor = anchor1,
                .modelOutput = modelOutput0->data.int8,
                .scale = ((TfLiteAffineQuantization *)(modelOutput0->quantization.params))->scale->data[0],
                .zeroPoint = ((TfLiteAffineQuantization *)(modelOutput0->quantization.params))->zero_point->data[0],
                .size = modelOutput0->bytes
            },
            Branch {
                .resolution = modelOutput1->dims->data[1],
                .numBox = 3,
                .anchor = anchor2,
                .modelOutput = modelOutput1->data.int8,
                .scale = ((TfLiteAffineQuantization *)(modelOutput1->quantization.params))->scale->data[0],
                .zeroPoint = ((TfLiteAffineQuantization *)(modelOutput1->quantization.params))->zero_point->data[0],
                .size = modelOutput1->bytes
            }
        },
        .topN = m_topN
    };
#endif
    /* End init */

    /* Start postprocessing */
    int originalImageWidth = imgSrcCols;
    int originalImageHeight = imgSrcRows;

    std::forward_list<image::Detection> detections;
    GetNetworkBoxes(net, originalImageWidth, originalImageHeight, m_threshold, detections);

    /* Do nms */
    CalculateNMS(detections, net.numClasses, m_nms);

    for (auto &it : detections)
    {
        float xMin = it.bbox.x - it.bbox.w / 2.0f;
        float xMax = it.bbox.x + it.bbox.w / 2.0f;
        float yMin = it.bbox.y - it.bbox.h / 2.0f;
        float yMax = it.bbox.y + it.bbox.h / 2.0f;

        if (xMin < 0)
        {
            xMin = 0;
        }

        if (yMin < 0)
        {
            yMin = 0;
        }

        if (xMax > originalImageWidth)
        {
            xMax = originalImageWidth;
        }

        if (yMax > originalImageHeight)
        {
            yMax = originalImageHeight;
        }

        float boxX = xMin;
        float boxY = yMin;
        float boxWidth = xMax - xMin;
        float boxHeight = yMax - yMin;

        for (int j = 0; j < net.numClasses; ++j)
        {
            if (it.prob[j] > 0)
            {

                DetectionResult tmpResult = {};
                tmpResult.m_normalisedVal = it.prob[j];
                tmpResult.m_x0 = (int)boxX;
                tmpResult.m_y0 = (int)boxY;
                tmpResult.m_w = (int)boxWidth;
                tmpResult.m_h = (int)boxHeight;
                tmpResult.m_cls = j;

                resultsOut.push_back(tmpResult);
            }
        }
    }

#if defined (__ICCARM__)
    net.branches.erase(net.branches.begin(), net.branches.end());
#endif
}


void DetectorPostprocessing::InsertTopNDetections(std::forward_list<image::Detection> &detections, image::Detection &det)
{
    std::forward_list<image::Detection>::iterator it;
    std::forward_list<image::Detection>::iterator last_it;

    for (it = detections.begin(); it != detections.end(); ++it)
    {
        if (it->objectness > det.objectness)
            break;

        last_it = it;
    }

    if (it != detections.begin())
    {
        detections.emplace_after(last_it, det);
        detections.pop_front();
    }
}

void DetectorPostprocessing::GetNetworkBoxes(Network &net, int imageWidth, int imageHeight, float threshold, std::forward_list<image::Detection> &detections)
{
    int numClasses = net.numClasses;
    int num = 0;
    auto det_objectness_comparator = [](image::Detection & pa, image::Detection & pb)
    {
        return pa.objectness < pb.objectness;
    };

    for (size_t i = 0; i < net.branches.size(); ++i)
    {
        int height   = net.branches[i].resolution;
        int width    = net.branches[i].resolution;
        int channel  = net.branches[i].numBox * (5 + numClasses);

        for (int h = 0; h < net.branches[i].resolution; h++)
        {
            for (int w = 0; w < net.branches[i].resolution; w++)
            {
                for (int anc = 0; anc < net.branches[i].numBox; anc++)
                {

                    /* Objectness score */
                    int bbox_obj_offset = h * width * channel + w * channel + anc * (numClasses + 5) + 4;
                    float objectness = math::MathUtils::SigmoidF32(
                                           (static_cast<float>(net.branches[i].modelOutput[bbox_obj_offset])
                                            - net.branches[i].zeroPoint
                                           ) * net.branches[i].scale);

                    if (objectness > threshold)
                    {
                        image::Detection det;
                        det.objectness = objectness;
                        /* Get bbox prediction data for each anchor, each feature point */
                        int bbox_x_offset = bbox_obj_offset - 4;
                        int bbox_y_offset = bbox_x_offset + 1;
                        int bbox_w_offset = bbox_x_offset + 2;
                        int bbox_h_offset = bbox_x_offset + 3;
                        int bbox_scores_offset = bbox_x_offset + 5;

                        det.bbox.x = (static_cast<float>(net.branches[i].modelOutput[bbox_x_offset]) - net.branches[i].zeroPoint) * net.branches[i].scale;
                        det.bbox.y = (static_cast<float>(net.branches[i].modelOutput[bbox_y_offset]) - net.branches[i].zeroPoint) * net.branches[i].scale;
                        det.bbox.w = (static_cast<float>(net.branches[i].modelOutput[bbox_w_offset]) - net.branches[i].zeroPoint) * net.branches[i].scale;
                        det.bbox.h = (static_cast<float>(net.branches[i].modelOutput[bbox_h_offset]) - net.branches[i].zeroPoint) * net.branches[i].scale;

                        float bbox_x, bbox_y;

                        /* Eliminate grid sensitivity trick involved in YOLOv4 */
                        bbox_x = math::MathUtils::SigmoidF32(det.bbox.x);
                        bbox_y = math::MathUtils::SigmoidF32(det.bbox.y);
                        det.bbox.x = (bbox_x + w) / width;
                        det.bbox.y = (bbox_y + h) / height;

                        det.bbox.w = std::exp(det.bbox.w) * net.branches[i].anchor[anc * 2] / net.inputWidth;
                        det.bbox.h = std::exp(det.bbox.h) * net.branches[i].anchor[anc * 2 + 1] / net.inputHeight;

                        for (int s = 0; s < numClasses; s++)
                        {
                            float sig = math::MathUtils::SigmoidF32(
                                            (static_cast<float>(net.branches[i].modelOutput[bbox_scores_offset + s]) -
                                             net.branches[i].zeroPoint) * net.branches[i].scale
                                        ) * objectness;
                            det.prob.emplace_back((sig > threshold) ? sig : 0);
                        }

                        /* Correct_YOLO_boxes */
                        det.bbox.x *= imageWidth;
                        det.bbox.w *= imageWidth;
                        det.bbox.y *= imageHeight;
                        det.bbox.h *= imageHeight;

                        if (num < net.topN || net.topN <= 0)
                        {
                            detections.emplace_front(det);
                            num += 1;
                        }
                        else if (num == net.topN)
                        {
                            detections.sort(det_objectness_comparator);
                            InsertTopNDetections(detections, det);
                            num += 1;
                        }
                        else
                        {
                            InsertTopNDetections(detections, det);
                        }
                    }
                }
            }
        }
    }

    if (num > net.topN)
        num -= 1;
}

} /* namespace object_detection */
} /* namespace app */
} /* namespace arm */
