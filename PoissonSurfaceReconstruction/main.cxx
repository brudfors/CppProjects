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

vtkSmartPointer<vtkPolyData> GetPointsFromCSV(const char* fileName);
bool TestUnorganizedPointsToSurface(vtkPolyData* points, int KNearestNeighbors = 3, int depth = 10,
                                    unsigned int graphType = vtkPointSetNormalOrientation::KNN_GRAPH);
bool TestPoissonFromEstimatedNormalsWithOrientation(vtkPolyData* points, int depth = 10);

int main(int argc, char *argv[])
{
  // Define reader and writer
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();

  // Read input data
  reader->SetFileName("H:/bin/TestSurfaceRecontruction/R/Data/SpherePoints.vtp");
  reader->Update();
  vtkSmartPointer<vtkPolyData> points = vtkSmartPointer<vtkPolyData>::New();
  points = reader->GetOutput();
  TestUnorganizedPointsToSurface(points);
  //points = reader->GetOutput();
  //vtkSmartPointer<vtkPolyData> pointsCSV = vtkSmartPointer<vtkPolyData>::New();
  //pointsCSV = GetPointsFromCSV("../Data/points.csv");
  //TestUnorganizedPointsToSurface(points, 3, 20);
  
  /*reader->SetFileName("../Data/ouput_normalOrientation.vtp");
  reader->Update();
  vtkSmartPointer<vtkPolyData> pointsNormalOrientation = vtkSmartPointer<vtkPolyData>::New();
  pointsNormalOrientation = reader->GetOutput();*/
  //TestPoissonFromEstimatedNormalsWithOrientation(pointsNormalOrientation, 5);

  getch();
  return 0;
}

bool TestPoissonFromEstimatedNormalsWithOrientation(vtkPolyData* points, int depth)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();

  if (points->GetNumberOfPoints() > 0)
  {
    std::cout << "STATUS: Poisson reconstruction..." << std::endl;
    vtkSmartPointer<vtkPoissonReconstruction> poissonFilter = vtkSmartPointer<vtkPoissonReconstruction>::New();
    poissonFilter->SetDepth(depth);
    poissonFilter->SetInputData(points);
    poissonFilter->Update();

    writer->SetInputData(poissonFilter->GetOutput());
    writer->SetFileName("H:/bin/TestSurfaceRecontruction/R/Data/ouput_poisson.vtp");
    writer->Update();
    return true;
  }
  else
  {
    std::cout << "ERROR: Input data contains zero points!" << std::endl;
    return false;
  }
}

bool TestUnorganizedPointsToSurface(vtkPolyData* points, int KNearestNeighbors, int depth, unsigned int graphType)
{
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();

  if (points->GetNumberOfPoints() > 0)
  {
    // Start timer
    vtkSmartPointer<vtkTimerLog> timer = vtkSmartPointer<vtkTimerLog>::New();
    timer->StartTimer();

    std::cout << "STATUS: Estimate normals..." << std::endl;
    vtkSmartPointer<vtkPointSetNormalEstimation> normalEstimation = vtkSmartPointer<vtkPointSetNormalEstimation>::New();
    normalEstimation->SetInputData(points);
    normalEstimation->Update();

    writer->SetInputData(normalEstimation->GetOutput());
    writer->SetFileName("H:/bin/TestSurfaceRecontruction/R/Data/ouput_normalEstimation.vtp");
    writer->Update();

    std::cout << "STATUS: Estimate normals orientation..." << std::endl;
    vtkSmartPointer<vtkPointSetNormalOrientation> normalOrientationFilter = vtkSmartPointer<vtkPointSetNormalOrientation>::New();
    normalOrientationFilter->SetInputData(normalEstimation->GetOutput());  
	  normalOrientationFilter->SetGraphFilterType(graphType);
    normalOrientationFilter->SetKNearestNeighbors(KNearestNeighbors);
    normalOrientationFilter->Update();

    writer->SetInputData(normalOrientationFilter->GetOutput());
    writer->SetFileName("H:/bin/TestSurfaceRecontruction/R/Data/ouput_normalOrientation.vtp");
    writer->Update();

    std::cout << "STATUS: Poisson reconstruction..." << std::endl;
    vtkSmartPointer<vtkPoissonReconstruction> poissonFilter = vtkSmartPointer<vtkPoissonReconstruction>::New();
    poissonFilter->SetDepth(depth);
    poissonFilter->SetInputData(normalOrientationFilter->GetOutput());
    poissonFilter->Update();

    writer->SetInputData(poissonFilter->GetOutput());
    writer->SetFileName("H:/bin/TestSurfaceRecontruction/R/Data/ouput_poisson.vtp");
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