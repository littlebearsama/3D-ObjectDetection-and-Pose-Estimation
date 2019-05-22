/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkImageNormalizedCrossCorrelation.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// http://www.mathworks.com/help/toolbox/images/ref/normxcorr2.html
// http://www.mathworks.com/help/toolbox/images/ref/corr2.html

#include "vtkImageNormalizedCrossCorrelation.h"

#include "vtkSmartPointer.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkObjectFactory.h"
#include "vtkImageData.h"
#include "vtkImageCast.h"
#include "vtkImageExtractComponents.h"
#include "vtkImageAppendComponents.h"
#include "vtkImageShiftScale.h"
#include "vtkImageMathematics.h"
#include "vtkExtractVOI.h"

#include <vector>

vtkStandardNewMacro(vtkImageNormalizedCrossCorrelation);

vtkImageNormalizedCrossCorrelation::vtkImageNormalizedCrossCorrelation()
{
  this->SetNumberOfInputPorts(2);
}

int vtkImageNormalizedCrossCorrelation::RequestData(vtkInformation *vtkNotUsed(request),
                               vtkInformationVector **inputVector,
                               vtkInformationVector *outputVector)
{
  // Get the info objects
  vtkInformation *image1Info = inputVector[0]->GetInformationObject(0);
  vtkInformation *image2Info = inputVector[1]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);

  // Get the input and ouptut
  vtkImageData *input1 = vtkImageData::SafeDownCast(
      image1Info->Get(vtkDataObject::DATA_OBJECT()));

  vtkImageData *input2 = vtkImageData::SafeDownCast(
      image2Info->Get(vtkDataObject::DATA_OBJECT()));

  vtkImageData *output = vtkImageData::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));


  return 1;
}

double vtkImageNormalizedCrossCorrelation::SubtractMean(vtkImageData* image)
{
  // This function expects the image type to be double and to have 1 scalar component

  vtkSmartPointer<vtkImageData> input =
    vtkSmartPointer<vtkImageData>::New();
  input->DeepCopy(image);

  if(image->GetNumberOfScalarComponents() > 1)
    {
    std::cerr << "Input to ComputeMean must be 1 component!" << std::endl;
    return -1;
    }

  if(image->GetScalarType() != VTK_DOUBLE)
    {
    std::cerr << "Input to ComputeMean must have scalar type double!" << std::endl;
    return -1;
    }
    
  double mean = ComputeMean(image);

  vtkSmartPointer<vtkImageShiftScale> shiftScaleFilter =
    vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScaleFilter->SetOutputScalarTypeToUnsignedChar();
  //shiftScaleFilter->SetInputConnection(input->GetProducerPort());
  shiftScaleFilter->SetInputData(input);
  shiftScaleFilter->SetShift(-mean);
  shiftScaleFilter->Update();

  image->DeepCopy(shiftScaleFilter->GetOutput());
}

double vtkImageNormalizedCrossCorrelation::ComputeMean(vtkImageData* image)
{
  // This function expects the image type to be double and to have 1 scalar component
  
  if(image->GetNumberOfScalarComponents() > 1)
    {
    std::cerr << "Input to ComputeMean must be 1 component!" << std::endl;
    return -1;
    }

  if(image->GetScalarType() != VTK_DOUBLE)
    {
    std::cerr << "Input to ComputeMean must have scalar type double!" << std::endl;
    return -1;
    }
  
  int extent[6];
  image->GetExtent(extent);

  double pixelSum = 0;
  unsigned int pixelCount = 0;

  for(unsigned int i = extent[0]; i <= extent[1]; i++)
    {
    for(unsigned int j = extent[2]; j <= extent[3]; j++)
      {
      for(unsigned int k = extent[4]; k <= extent[5]; k++)
        {
        double* pixel = static_cast<double*>(image->GetScalarPointer(i,j,k));
        pixelSum += pixel[0];
        pixelCount++;
        }
      }
    }

  return pixelSum/static_cast<double>(pixelCount);
}


void vtkImageNormalizedCrossCorrelation::CrossCorrelationColor(vtkImageData* image, vtkImageData* patch, vtkImageData* output)
{
  // This should be moved to RequestData

  std::vector<vtkSmartPointer<vtkImageData> > outputComponents;

  vtkSmartPointer<vtkImageAppendComponents> appendFilter =
    vtkSmartPointer<vtkImageAppendComponents>::New();
    
  for(vtkIdType c = 0; c < image->GetNumberOfScalarComponents(); c++)
    {
    std::cout << "Computing correlation of component " << c << "..." << std::endl;
  
    outputComponents.push_back(vtkSmartPointer<vtkImageData>::New());
  
    vtkSmartPointer<vtkImageExtractComponents> imageExtractComponents =
      vtkSmartPointer<vtkImageExtractComponents>::New();
    //imageExtractComponents->SetInputConnection(image->GetProducerPort());
	imageExtractComponents->SetInputData(image);
    imageExtractComponents->SetComponents(c);
    imageExtractComponents->Update();

    vtkSmartPointer<vtkImageExtractComponents> patchExtractComponents =
      vtkSmartPointer<vtkImageExtractComponents>::New();
    //patchExtractComponents->SetInputConnection(patch->GetProducerPort());
	patchExtractComponents->SetInputData(patch);
    patchExtractComponents->SetComponents(c);
    patchExtractComponents->Update();

    CrossCorrelationGreyscale(imageExtractComponents->GetOutput(), patchExtractComponents->GetOutput(), outputComponents[c]);
    //appendFilter->AddInputConnection(0, outputComponents[c]->GetProducerPort());
	appendFilter->AddInputData(0, outputComponents[c]);
    }

  appendFilter->Update();

  output->ShallowCopy(appendFilter->GetOutput());
}

void vtkImageNormalizedCrossCorrelation::CrossCorrelationGreyscale(vtkImageData* image, vtkImageData* patch, vtkImageData* output)
{
  if(image->GetNumberOfScalarComponents() > 1)
    {
    std::cerr << "Input to ComputeMean must be 1 component!" << std::endl;
    return;
    }

  output->SetExtent(image->GetExtent());

  int patchDimensions[6];
  patch->GetDimensions(patchDimensions);

  int imageExtent[6];
  image->GetExtent(imageExtent);

  // Cast the image and the patch to doubles
  vtkSmartPointer<vtkImageCast> imageCastFilter =
    vtkSmartPointer<vtkImageCast>::New();
  //imageCastFilter->SetInputConnection(image->GetProducerPort());
  imageCastFilter->SetInputData(image);
  imageCastFilter->SetOutputScalarTypeToDouble();
  imageCastFilter->Update();

  vtkSmartPointer<vtkImageCast> patchCastFilter =
    vtkSmartPointer<vtkImageCast>::New();
  //patchCastFilter->SetInputConnection(patch->GetProducerPort());
  patchCastFilter->SetInputData(patch);
  patchCastFilter->SetOutputScalarTypeToDouble();
  patchCastFilter->Update();

  // Normalize the patch
  vtkSmartPointer<vtkImageData> normalizedPatch =
    vtkSmartPointer<vtkImageData>::New();
  normalizedPatch->DeepCopy(patchCastFilter->GetOutput());
  //NormalizeImage(normalizedPatch);
  SubtractMean(normalizedPatch);

  vtkSmartPointer<vtkImageMathematics> squarePatch =
    vtkSmartPointer<vtkImageMathematics>::New();
  squarePatch->SetOperationToSquare();
  //squarePatch->SetInputConnection(normalizedPatch->GetProducerPort());
  squarePatch->SetInputData(normalizedPatch);
  squarePatch->Update();

  double patchSquareSum = PixelSum(squarePatch->GetOutput());
  
  // (i,j) is the current center of the kernel
  /*
  for(int i = imageExtent[0] + patchDimensions[0]; i < imageExtent[1] - patchDimensions[0]; i++)
    {
    for(int j = imageExtent[2] + patchDimensions[1]; j < imageExtent[3] - patchDimensions[1]; j++)
      {
      */
  for(int i = 10; i < 20; i++)
    {
    for(int j = 10; j < 11; j++)
      {
      std::cout << "(" << i << " , " << j << ")" << std::endl;
      // Extract relevant region of image1Info
      vtkSmartPointer<vtkExtractVOI> extractVOI =
        vtkSmartPointer<vtkExtractVOI>::New();
      extractVOI->SetInputConnection(imageCastFilter->GetOutputPort());
      extractVOI->SetVOI(i - patchDimensions[0]/2, i + patchDimensions[0]/2,
                         j - patchDimensions[1]/2, j + patchDimensions[1]/2, 0, 0);
      extractVOI->Update();
      
      // Normalize extracted region of the image
      vtkSmartPointer<vtkImageData> normalizedImage =
        vtkSmartPointer<vtkImageData>::New();
      normalizedImage->DeepCopy(imageCastFilter->GetOutput());
      //NormalizeImage(normalizedImage);
      SubtractMean(normalizedImage);

      // Compute numerator of corr2 equation
      vtkSmartPointer<vtkImageMathematics> multiply =
        vtkSmartPointer<vtkImageMathematics>::New();
      multiply->SetOperationToMultiply();
      //multiply->SetInput1(normalizedPatch);
	  multiply->SetInput1Data(normalizedPatch);
      //multiply->SetInput2(normalizedImage);
	  multiply->SetInput2Data(normalizedImage);
      multiply->Update();

      double numeratorSum = PixelSum(multiply->GetOutput());
      
      // Square the image and the patch
      vtkSmartPointer<vtkImageMathematics> squareImage =
        vtkSmartPointer<vtkImageMathematics>::New();
      squareImage->SetOperationToSquare();
      //squareImage->SetInputConnection(normalizedImage->GetProducerPort());
	  squareImage->SetInputData(normalizedImage);
      squareImage->Update();

      double imageSquareSum = PixelSum(squareImage->GetOutput());
      
      double correlation = sqrt(imageSquareSum * patchSquareSum);

      double* pixel = static_cast<double*>(output->GetScalarPointer(i,j,0));
      pixel[0] = correlation;
      }
    }
  
}

void vtkImageNormalizedCrossCorrelation::NormalizeImage(vtkImageData* image)
{
  vtkSmartPointer<vtkImageData> input =
    vtkSmartPointer<vtkImageData>::New();
  input->DeepCopy(image);

  // Compute and subtract the mean
  SubtractMean(input);

  image->ShallowCopy(input);
}

double vtkImageNormalizedCrossCorrelation::PixelSum(vtkImageData* image)
{
  double pixelSum = 0;

  int extent[6];
  image->GetExtent(extent);

  for(unsigned int i = extent[0]; i <= extent[1]; i++)
    {
    for(unsigned int j = extent[2]; j <= extent[3]; j++)
      {
      for(unsigned int k = extent[4]; k <= extent[5]; k++)
        {
        double* pixel = static_cast<double*>(image->GetScalarPointer(i,j,k));
        pixelSum += pixel[0];
        }
      }
    }

  return pixelSum;
}

void vtkImageNormalizedCrossCorrelation::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}