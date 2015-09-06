#include <iostream>
#include <conio.h>

#include <vtkTimerLog.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>

#include "vtkPointSetNormalEstimation.h"
#include "vtkPoissonReconstruction.h"
#include "vtkPointSetNormalOrientation.h"

#define MAIN_RETURN getch(); return 0;

std::string dataFolderPath = "H:/_B/CppProjects/PoissonSurfaceReconstructionD/Data/"; // TODO: Relative paths are not working for some reason

vtkSmartPointer<vtkPolyData> GetPointsFromCSV(const char* fileName);
bool TestUnorganizedPointsToSurface(vtkPolyData* points, int KNearestNeighbors = 3, int depth = 10,
                                    unsigned int graphType = vtkPointSetNormalOrientation::KNN_GRAPH);
bool TestPoissonFromEstimatedNormalsWithOrientation(vtkPolyData* points, int depth = 10);

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();

  std::string path = dataFolderPath + "RecordedModel_2015-09-04.vtp"; // RecordedModel_2015-09-04.vtp SpherePoints.vtp
  reader->SetFileName(path.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
  points = reader->GetOutput();
  TestUnorganizedPointsToSurface(points);

  MAIN_RETURN
}

bool TestUnorganizedPointsToSurface(vtkPolyData* points, int KNearestNeighbors, int depth, unsigned int graphType)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();

  if (points->GetNumberOfPoints() > 0)
  {
    std::string path;

    // Start timer
    vtkSmartPointer<vtkTimerLog> timer = vtkSmartPointer<vtkTimerLog>::New();
    timer->StartTimer();

    std::cout << "STATUS: Estimate normals..." << std::endl;
    vtkSmartPointer<vtkPointSetNormalEstimation> normalEstimation = vtkSmartPointer<vtkPointSetNormalEstimation>::New();
    normalEstimation->SetInputData(points);
    normalEstimation->Update();

    writer->SetInputData(normalEstimation->GetOutput());
    path = dataFolderPath + "ouput_normalEstimation.vtp";
    writer->SetFileName(path.c_str());
    writer->Update();

    std::cout << "STATUS: Estimate normals orientation..." << std::endl;
    vtkSmartPointer<vtkPointSetNormalOrientation> normalOrientationFilter = vtkSmartPointer<vtkPointSetNormalOrientation>::New();
    normalOrientationFilter->SetInputData(normalEstimation->GetOutput());  
	  normalOrientationFilter->SetGraphFilterType(graphType);
    normalOrientationFilter->SetKNearestNeighbors(KNearestNeighbors);
    normalOrientationFilter->Update();

    writer->SetInputData(normalOrientationFilter->GetOutput());
    path = dataFolderPath + "ouput_normalOrientation.vtp";
    writer->SetFileName(path.c_str());
    writer->Update();

    std::cout << "STATUS: Poisson reconstruction..." << std::endl;
    vtkSmartPointer<vtkPoissonReconstruction> poissonFilter = vtkSmartPointer<vtkPoissonReconstruction>::New();
    poissonFilter->SetDepth(depth);
    poissonFilter->SetInputData(normalOrientationFilter->GetOutput());
    poissonFilter->Update();

    writer->SetInputData(poissonFilter->GetOutput());
    path = dataFolderPath + "ouput_poisson.vtp";
    writer->SetFileName(path.c_str());
    writer->Update();

    // Output runtime
    timer->StopTimer();
    std::cout << "STATUS: Elapsed time: " << 	timer->GetElapsedTime() << std::endl;
    return true;
  }
  else
  {
    std::cout << "ERROR: Input data contains zero points!" << std::endl;
    return false;
  }
}

bool TestPoissonFromEstimatedNormalsWithOrientation(vtkPolyData* points, int depth)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();

  if (points->GetNumberOfPoints() > 0)
  {
    std::string path;

    std::cout << "STATUS: Poisson reconstruction..." << std::endl;
    vtkSmartPointer<vtkPoissonReconstruction> poissonFilter = vtkSmartPointer<vtkPoissonReconstruction>::New();
    poissonFilter->SetDepth(depth);
    poissonFilter->SetInputData(points);
    poissonFilter->Update();

    writer->SetInputData(poissonFilter->GetOutput());
    path = dataFolderPath + "ouput_poisson.vtp";
    writer->SetFileName(path.c_str());
    writer->Update();
    return true;
  }
  else
  {
    std::cout << "ERROR: Input data contains zero points!" << std::endl;
    return false;
  }
}

// Reads a CSV file containing x,y,z coordinates on each row delimited by a ','
vtkSmartPointer<vtkPolyData> GetPointsFromCSV(const char* fileName)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  // Read CSV file
  ifstream csvFile(fileName);
  std::string line;
  if (csvFile.is_open())
  {
    while(std::getline(csvFile, line))
    {
      std::istringstream s(line);
      std::string field;
      float xyz[3];
      int idx = 0;
      while (getline(s, field,','))
      {
        xyz[idx] = atof(field.c_str());
        ++idx;					
      }
      points->InsertNextPoint(xyz[0], xyz[1], xyz[2]);
    }
    csvFile.close();
  }

  vtkSmartPointer<vtkPolyData> model = vtkSmartPointer<vtkPolyData>::New();
  model->SetPoints(points);

  // This filter throws away all of the cells in the input and replaces them with a vertex on each point.
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(model);
  glyphFilter->Update();

  return glyphFilter->GetOutput();
}