/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkImageNormalizedCrossCorrelation.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkImageNormalizedCrossCorrelation - Reads common image formats.
// .SECTION Description
// vtkImageNormalizedCrossCorrelation computes the normalized cross correlation
// between two images.

#ifndef __vtkImageNormalizedCrossCorrelation_h
#define __vtkImageNormalizedCrossCorrelation_h

#include "vtkImageAlgorithm.h"

class vtkImageNormalizedCrossCorrelation : public vtkImageAlgorithm
{
public:
  static vtkImageNormalizedCrossCorrelation *New();
  vtkTypeMacro(vtkImageNormalizedCrossCorrelation,vtkImageAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

  // Description:
  // This function performs normalized cross correlation. It should be done in RequestData,
  // but currently having trouble with pipeline (whole extents, etc)
  void CrossCorrelationColor(vtkImageData* image, vtkImageData* patch, vtkImageData* output);

protected:
  vtkImageNormalizedCrossCorrelation();

  // Description:
  // Normalized cross correlation of a single component image
  void CrossCorrelationGreyscale(vtkImageData* image, vtkImageData* patch, vtkImageData* output);
  
  // Description:
  // This function computes the sum of all pixels in an image.
  double PixelSum(vtkImageData*);

  // Description:
  // This function computes the mean of a 1 component image.
  double ComputeMean(vtkImageData*);

  // Description:
  // This function computes and subtracts the mean from an image.
  void NormalizeImage(vtkImageData* input);

  // Description:
  // This function subtracts the mean from the image. This is done one
  // component at a time.
  double SubtractMean(vtkImageData*);

private:
  vtkImageNormalizedCrossCorrelation(const vtkImageNormalizedCrossCorrelation&);  // Not implemented.
  void operator=(const vtkImageNormalizedCrossCorrelation&);  // Not implemented.

};

#endif