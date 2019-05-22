#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkImageActor.h>
#include <vtkPNGReader.h>
#include <vtkImageCorrelation.h>
#include <vtkExtractVOI.h>

#include "vtkImageNormalizedCrossCorrelation.h"

int main(int argc, char *argv[])
{
  // Verify command line arguments
  if(argc != 2)
    {
    std::cerr << "Required arguments: InputFilename.png" << std::endl;
    return EXIT_FAILURE;
    }

  vtkSmartPointer<vtkPNGReader> reader =
    vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();

  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  //originalActor->SetInput(reader->GetOutput());
  originalActor->SetInputData(reader->GetOutput());

  vtkSmartPointer<vtkExtractVOI> extractPatch =
    vtkSmartPointer<vtkExtractVOI>::New();
  extractPatch->SetInputConnection(reader->GetOutputPort());
  int demoCenter[2] = {20, 20};
  int kernelRadius = 5;
  extractPatch->SetVOI(demoCenter[0] - kernelRadius, demoCenter[0] + kernelRadius,
                       demoCenter[1] - kernelRadius, demoCenter[1] + kernelRadius, 0, 0);
  extractPatch->Update();

  vtkSmartPointer<vtkImageData> patch =
    vtkSmartPointer<vtkImageData>::New();
  patch->DeepCopy(extractPatch->GetOutput());

  /* Filter-style
  vtkSmartPointer<vtkImageNormalizedCrossCorrelation> normalizedCorrelationFilter =
    vtkSmartPointer<vtkImageNormalizedCrossCorrelation>::New();
  normalizedCorrelationFilter->SetInputConnection(0, reader->GetOutputPort());
  //normalizedCorrelationFilter->SetInputConnection(1, extractPatch->GetOutputPort());
  normalizedCorrelationFilter->SetInputConnection(1, patch->GetProducerPort());
  normalizedCorrelationFilter->Update();
  */


  // Function-style
  vtkSmartPointer<vtkImageData> correlationImage =
    vtkSmartPointer<vtkImageData>::New();
    
  vtkSmartPointer<vtkImageNormalizedCrossCorrelation> normalizedCorrelationFilter =
    vtkSmartPointer<vtkImageNormalizedCrossCorrelation>::New();
  normalizedCorrelationFilter->CrossCorrelationColor(reader->GetOutput(), extractPatch->GetOutput(), correlationImage);
  
  vtkSmartPointer<vtkImageActor> normalizedCorrelationActor =
    vtkSmartPointer<vtkImageActor>::New();
  //normalizedCorrelationActor->SetInput(normalizedCorrelationFilter->GetOutput());
  normalizedCorrelationActor->SetInputData(normalizedCorrelationFilter->GetOutput());

  vtkSmartPointer<vtkImageCorrelation> correlationFilter =
    vtkSmartPointer<vtkImageCorrelation>::New();
  //correlationFilter->SetInput1(reader->GetOutput());
  correlationFilter->SetInput1Data(reader->GetOutput());
  //correlationFilter->SetInput2(extractPatch->GetOutput());
  correlationFilter->SetInput2Data(extractPatch->GetOutput());
  correlationFilter->Update();

  // Create an actor
  vtkSmartPointer<vtkImageActor> standardCorrelationActor =
    vtkSmartPointer<vtkImageActor>::New();
  //standardCorrelationActor->SetInput(correlationFilter->GetOutput());
  standardCorrelationActor->SetInputData(correlationFilter->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double originalViewport[4] = {0.0, 0.0, 0.33, 1.0};
  double standardCorrelationViewport[4] = {0.33, 0.0, 0.66, 1.0};
  double normalizedCorrelationViewport[4] = {0.66, 0.0, 1.0, 1.0};

  // Setup render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(900, 300);

  // Setup renderers
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  originalRenderer->AddActor(originalActor);
  originalRenderer->ResetCamera();
  renderWindow->AddRenderer(originalRenderer);
  originalRenderer->SetViewport(originalViewport);

  vtkSmartPointer<vtkRenderer> standardCorrelationRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  standardCorrelationRenderer->AddActor(standardCorrelationActor);
  standardCorrelationRenderer->ResetCamera();
  renderWindow->AddRenderer(standardCorrelationRenderer);
  standardCorrelationRenderer->SetViewport(standardCorrelationViewport);

  vtkSmartPointer<vtkRenderer> normalizedCorrelationRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  normalizedCorrelationRenderer->AddActor(normalizedCorrelationActor);
  normalizedCorrelationRenderer->ResetCamera();
  renderWindow->AddRenderer(normalizedCorrelationRenderer);
  normalizedCorrelationRenderer->SetViewport(normalizedCorrelationViewport);

  // Setup render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  // Render and start interaction
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();
  
  getchar();
  return EXIT_SUCCESS;
}
