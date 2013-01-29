/*
 * Copyright (c) 2013, Fraunhofer FKIE
 *
 * Authors: Jochen Welle
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer FKIE nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * This file is part of the StructureColoring ROS package.
 *
 * The StructureColoring ROS package is free software:
 * you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The StructureColoring ROS package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with The StructureColoring ROS package.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <structureColoring/segcomp/rangeimageio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <map>


void usage(const char* prg)
{
    std::cout << "Usage: " << prg << " infilename outfilename\n";
    std::cout << "Convert the image \"infilename\" to a 8-bit grayscale SUN raster image \"outfilename\". ";
    std::cout << "The colors are converted as follows:\n";
    std::cout << "\tBlack\t-> 0\n"
        "\tWhite\t-> 1\n"
        "\tall other colors are given consecutive values starting from 10 upwards.\n"
        "Therefore more than 248 different colors in the input image are not allowed.\n";
}

uint32_t rgbToInt(const cv::Vec3b& rgb)
{
    return ((rgb[0] << 16) | (rgb[1] << 8) | (rgb[2]));
}
cv::Vec3b intToRgb(uint32_t i)
{
    cv::Vec3b rgb;
    rgb[0] = (i >> 16) & 255;
    rgb[1] = (i >> 8) & 255;
    rgb[2] = i & 255;
    return rgb;
}

std::ostream& operator<< (std::ostream& os, const cv::Vec3b& v)
{
    os << (int)v[0] << "," << (int)v[1] << "," << (int)v[2];
    return os;
}

int main(int argc, char* argv[]) 
{
    typedef std::map<uint32_t, uint8_t> ColorMap;

    if (argc < 3)
    {
        usage(argv[0]);
    }
    else
    {
        cv::Mat input = cv::imread(argv[1], 1); 
        unsigned int width = input.cols;
        unsigned int height = input.rows;
        std::cout << argv[1] << ": " << width << " x " << height << "\n";
        std::vector<unsigned char> data;
        ColorMap colorMap;
        uint8_t segCount = 10;
        for (size_t j = 0; j < height; ++j)
        {
            for (size_t i = 0; i < width; ++i)
            {
                //uint8_t c = input.at<uint8_t>(j,i);
                cv::Vec3b rgb = input.at<cv::Vec3b>(j,i);
                uint32_t rgbInt = rgbToInt(rgb);
                if ((rgb[0] == 255) && (rgb[1] == 255) && (rgb[2] == 255))
                    colorMap[rgbInt] = 1;
                else if (rgbInt == 0)
                    colorMap[rgbInt] = 0;
                ColorMap::iterator it = colorMap.find(rgbInt);
                if (it == colorMap.end()) // not seen yet
                {
                    colorMap[rgbInt] = segCount;
                    if (segCount < 255)
                        ++segCount;
                    else
                    {
                        std::cerr << "ERROR: input image has more than 248 colors!" << std::endl;
                        exit(1);
                    }
                }
                data.push_back(colorMap[rgbInt]);
            }
        }
        for (ColorMap::iterator it = colorMap.begin(); it != colorMap.end(); ++it)
            std::cout << intToRgb(it->first) << ": " << (int)it->second << std::endl;
        std::cout << "map size: " << colorMap.size() << std::endl;
        writeRasterfile(argv[2], width, height, data);
    }
    return 0;
}

