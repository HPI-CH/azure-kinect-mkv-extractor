# Azure Kinect MKV Extractor

## Introduction

This tool is able to extract various data sources from a single MKV file that was recorded with the Azure Kinect MKV Recorder tool. 

https://docs.microsoft.com/de-de/azure/kinect-dk/azure-kinect-recorder

Currently, it is able to extract the following data:

* 3D Joint Coordinates and Orientations
* 2D Joint Coordinates
* Individual RGB images, named with the current timestamps
* 3D Colored Pointclouds as ply format (takes longer to process)

## Usage Info

The extraction tool acceppts the following arguments. After the input file, the order does not matter. 

```
offline_processor.exe <input_mkv_file> [<--gpu>, <--skeleton>, <--rgb>, <--pointcloud>]
```
