/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file SVMFileCreator.cpp
 * @author Andreas Richtsfeld, Ekaterina Potapova
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#include "SVMFileCreator.h"

namespace svm
{

/**
 * @brief Constructor of SVMFileCreator
 */
SVMFileCreator::SVMFileCreator()
{
  analyze = false;
  testset = false;
  have_relations = false;
  have_surfaces = false;
  featureNumber = -1;
}


/**
 * @brief Destructor of SVMFileCreator
 */
SVMFileCreator::~SVMFileCreator()
{}

/**
 * @brief Set input view with surfaces.
 */
void SVMFileCreator::setRelations(std::vector<surface::Relation> _relations)
{
  relations = _relations;
  have_relations = true;
}

void SVMFileCreator::setSurfaces(std::vector<surface::SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
}

/**
 * @brief Process the relation extraction algorithm
 */
void SVMFileCreator::process()
{
  if( (!have_relations) || (!have_surfaces) ) {
    std::printf("[SVMFileCreator::process] Error: Relations and surfaces are not available.\n");
    return;
  }

  //if we want to create feature file to learn
  if(!testset)
  {
    std::string current_file_name;
    FILE *features_file  = std::fopen(filename_base.c_str(), "a");        // first level results
    FILE *features_file_as = std::fopen(filename_base_as.c_str(), "a");      // second level results

    current_file_name = filename_base + ".pos";
    FILE *positive_file = std::fopen(current_file_name.c_str(), "a");    // first level positiv results
    current_file_name = filename_base + ".neg";
    FILE *negative_file = std::fopen(current_file_name.c_str(), "a");    // first level negative results
    current_file_name = filename_base_as + ".pos";
    FILE *positive_file_as = std::fopen(current_file_name.c_str(), "a");    // second level positiv results
    current_file_name = filename_base_as + ".neg";
    FILE *negative_file_as = std::fopen(current_file_name.c_str(), "a");    // second level negative results

    //@ep: this seems to be a hack!!!
    size_t min_size_1st = 30;           /// remove relations of small surfaces! (<30 pts for structural level)
    size_t min_size_2nd = 300;          /// remove relations of small surfaces! (<300 pts for assembly level)
    std::vector<bool> usedRelations;
    usedRelations.resize(relations.size());
    //select relations for training
    for(unsigned int i = 0; i < relations.size(); i++)
    {
      usedRelations.at(i) = false;
      
      if((surfaces.at(relations.at(i).id_0)->indices.size() > min_size_1st) &&
         (surfaces.at(relations.at(i).id_1)->indices.size() > min_size_1st) &&
         (relations.at(i).type == surface::STRUCTURAL_RELATIONS))
      {
        usedRelations.at(i) = true;
      }
      if((surfaces.at(relations.at(i).id_0)->indices.size() > min_size_2nd) &&
         (surfaces.at(relations.at(i).id_1)->indices.size() > min_size_2nd) &&
         (relations.at(i).type == surface::ASSEMBLY_RELATIONS))
      {
        usedRelations.at(i) = true;
      }
    }
    
    for(unsigned int i = 0; i < relations.size(); i++)
    {

      if(!(usedRelations.at(i)))
        continue;

      // structural relations
      if( (relations.at(i).type ==surface::STRUCTURAL_RELATIONS) && (relations.at(i).groundTruth != -1) )
      {
        std::fprintf(features_file,"%u ", relations.at(i).groundTruth);
        
        //write all relations
        if(featureNumber == -1)
	{
          for(unsigned int j = 0; j < relations.at(i).rel_value.size(); j++) 
	  {
            if( std::isnan(relations.at(i).rel_value.at(j)) )
            {
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
            }
            std::fprintf(features_file,"%u:%6.5f ", j+1, relations.at(i).rel_value.at(j));
          }
	}
	else
	{
          //write specific relations
          if( std::isnan(relations.at(i).rel_value.at(featureNumber)) )
          {
            printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, featureNumber);
          }
          std::fprintf(features_file,"%u:%6.5f ", 1, relations.at(i).rel_value.at(featureNumber));
	}
	
	std::fprintf(features_file,"\n");
        
        //write to positive relations
        if(relations.at(i).groundTruth) 
	{
          std::fprintf(positive_file,"%u ", relations.at(i).groundTruth);
	  
	  if(featureNumber == -1)
	  {
            for(unsigned int j = 0; j < relations.at(i).rel_value.size(); j++) 
	    {
              if( std::isnan(relations.at(i).rel_value.at(j)) )
              {
                printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
              }
              std::fprintf(positive_file,"%u:%6.5f ", j+1, relations.at(i).rel_value.at(j));
            }
	  }
	  else
	  {
	    if( std::isnan(relations.at(i).rel_value.at(featureNumber)) )
            {
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, featureNumber);
            }
            std::fprintf(positive_file,"%u:%6.5f ", 1, relations.at(i).rel_value.at(featureNumber));
	  }
	  std::fprintf(positive_file,"\n");
        }
        else 
	{
          std::fprintf(negative_file,"%u ", relations.at(i).groundTruth);
          
	  if(featureNumber == -1)
	  {
            for(unsigned int j = 0; j < relations.at(i).rel_value.size(); j++) 
	    {
              if( std::isnan(relations.at(i).rel_value.at(j)) )
              {
                printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
              }
              std::fprintf(negative_file,"%u:%6.5f ", j+1, relations.at(i).rel_value.at(j));
            }
	  }
	  else
	  {
	    if( std::isnan(relations.at(i).rel_value.at(featureNumber)) )
            {
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, featureNumber);
            }
            std::fprintf(negative_file,"%u:%6.5f ", 1, relations.at(i).rel_value.at(featureNumber));
	  }
	  std::fprintf(negative_file,"\n");
        }    
      }

      // assembly relations
      else if( (relations.at(i).type == surface::ASSEMBLY_RELATIONS) && (relations.at(i).groundTruth != -1) )   
      {
        fprintf(features_file_as,"%u ", relations.at(i).groundTruth);
        
	if(featureNumber == -1)
	{
          for(unsigned int j = 0; j < relations.at(i).rel_value.size(); j++) 
	  {
            if( std::isnan(relations.at(i).rel_value.at(j)) )
            {
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
            }
            fprintf(features_file_as,"%u:%6.5f ", j+1, relations.at(i).rel_value.at(j));
          }
	}
	else
	{
	  if( std::isnan(relations.at(i).rel_value.at(featureNumber)) )
          {
            printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, featureNumber);
          }
          fprintf(features_file_as,"%u:%6.5f ", 1, relations.at(i).rel_value.at(featureNumber));
	}
	fprintf(features_file_as,"\n");
      }

      if( (relations.at(i).type != surface::STRUCTURAL_RELATIONS) && (relations.at(i).type != surface::ASSEMBLY_RELATIONS) )
      {
        printf("[SVMFileCreator::process] Warning: Unknown type of relation received.\n");
      }
    }
    std::fclose(features_file);
    std::fclose(features_file_as);
    std::fclose(positive_file);
    std::fclose(positive_file_as);
    std::fclose(negative_file);
    std::fclose(negative_file_as);

    
//     if(analyze)
//     {
//       static int counter = 0;
//       
//       FILE *PPfileAna = std::fopen("./PP-Trainingsset.txt.ana", "a");
//       FILE *PPfilePos = std::fopen("./PP-Trainingsset.txt.pos", "a");
//       FILE *PPfileNeg = std::fopen("./PP-Trainingsset.txt.neg", "a");
//       FILE *PP2fileAna = std::fopen("./PP2-Trainingsset.txt.ana", "a");
//       FILE *PP2filePos = std::fopen("./PP2-Trainingsset.txt.pos", "a");
//       FILE *PP2fileNeg = std::fopen("./PP2-Trainingsset.txt.neg", "a");
//       
//       FILE *PPfileAnaRel = std::fopen("./PP-Trainingsset.txt.rel.ana", "a");
//       FILE *PP2fileAnaRel = std::fopen("./PP2-Trainingsset.txt.rel.ana", "a");
//       
//       for(int i=0; i<relations.size(); i++)
//       {
//         if(relations.at(i).type == 1)
//         {
//           std::fprintf(PPfileAna,"%u, ", relations.at(i).groundTruth);
// 	  
// 	  if(featureNumber == -1)
// 	  {
//             for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PPfileAna,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	  }
// 	  else
// 	  {
// 	    std::fprintf(PPfileAna,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	  }
//           std::fprintf(PPfileAna,"\n");
//           
//           std::fprintf(PPfileAnaRel,"%u: [%u][%u] (%u), ", counter, relations.at(i).id_0, relations.at(i).id_1, relations.at(i).groundTruth);
//           
// 	  if(featureNumber == -1)
// 	  {
// 	    for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PPfileAnaRel,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	  }
// 	  else
// 	  {
// 	    std::fprintf(PPfileAnaRel,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	  }
//           std::fprintf(PPfileAnaRel,"\n");          
//           
//           if(relations.at(i).groundTruth) {
// 	    
// 	    if(featureNumber == -1)
// 	    {
//               for(int j=0; j<relations.at(i).rel_value.size(); j++)
//                 std::fprintf(PPfilePos,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	    }
// 	    else
// 	    {
// 	      std::fprintf(PPfilePos,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	    }
//             std::fprintf(PPfilePos,"\n");
//           }
//           else {
//             if(featureNumber == -1)
// 	    {
// 	      for(int j=0; j<relations.at(i).rel_value.size(); j++)
//                 std::fprintf(PPfileNeg,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	    }
// 	    else
// 	    {
// 	      std::fprintf(PPfileNeg,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	    }
//             std::fprintf(PPfileNeg,"\n");
//           }
//         }
//         if(relations.at(i).type == 2)
//         {
//           std::fprintf(PP2fileAna,"%u, ", relations.at(i).groundTruth);
//           
// 	  if(featureNumber == -1)
// 	  {
// 	    for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PP2fileAna,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	  }
// 	  else
// 	  {
// 	    std::fprintf(PP2fileAna,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	  }
//           std::fprintf(PP2fileAna,"\n");
//   
//           std::fprintf(PP2fileAnaRel,"%u: [%u][%u] (%u), ", counter, relations.at(i).id_0, relations.at(i).id_1, relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++)
//             std::fprintf(PP2fileAnaRel,"%6.5f, ", relations.at(i).rel_value.at(j));
//           std::fprintf(PP2fileAnaRel,"\n");   
//           
//           if(relations.at(i).groundTruth) {
//             if(featureNumber == -1)
// 	    {
// 	      for(int j=0; j<relations.at(i).rel_value.size(); j++)
//                 std::fprintf(PP2filePos,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	    }
// 	    else
// 	    {
// 	      std::fprintf(PP2filePos,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	    }
//             std::fprintf(PP2filePos,"\n");
//           }
//           else {
//             if(featureNumber == -1)
// 	    {
// 	      for(int j=0; j<relations.at(i).rel_value.size(); j++)
//                 std::fprintf(PP2fileNeg,"%6.5f, ", relations.at(i).rel_value.at(j));
// 	    }
// 	    else
// 	    {
// 	      std::fprintf(PP2fileNeg,"%6.5f, ", relations.at(i).rel_value.at(featureNumber));
// 	    }
//             std::fprintf(PP2fileNeg,"\n");
//           }
//         }
//       }
//       std::fclose(PPfileAna);
//       std::fclose(PPfilePos);
//       std::fclose(PPfileNeg);
//       std::fclose(PP2fileAna);
//       std::fclose(PP2filePos);
//       std::fclose(PP2fileNeg);
//       std::fclose(PPfileAnaRel);
//       std::fclose(PP2fileAnaRel);
//       counter++;
//     } 
  }
  
//   /// TESTSET OUTPUT
//   /// The output of the testset is not complete (write only relations, if they are correct or not)
//   else
//   {
//     
//     size_t min_size_1st = 0;
//     size_t min_size_2st = 0;
//     for(int i=0; i<view->relations.size(); i++) 
//     {
//       if(view->surfaces[view->relations.at(i).id_0]->indices.size() > min_size_1st &&
//          view->surfaces[view->relations.at(i).id_1]->indices.size() > min_size_1st &&
//          view->relations.at(i).type == 1)
//         relations.push_back(view->relations.at(i));
//       if(view->surfaces[view->relations.at(i).id_0]->indices.size() > min_size_2st &&
//          view->surfaces[view->relations.at(i).id_1]->indices.size() > min_size_2st &&
//          view->relations.at(i).type == 2)
//         relations.push_back(view->relations.at(i));
//     }
//     
//     static int tp = 0, tp2 = 0;
//     static int tn = 0, tn2 = 0;
//     static int fp = 0, fp2 = 0;
//     static int fn = 0, fn2 = 0;
//     
//     FILE *PPfileAnaNew = std::fopen("./PP-Analyze.txt", "a");           // first level results
//     FILE *PPfileAnaNew2 = std::fopen("./PP2-Analyze.txt", "a");         // second level results
//     std::fprintf(PPfileAnaNew,"tp, tn, fp, fn, BER\n");
//     std::fprintf(PPfileAnaNew2,"tp, tn, fp, fn, BER\n");
//     for(int i=0; i<relations.size(); i++) 
//     {
//       if(relations.at(i).type == 1 && relations.at(i).groundTruth != -1) 
//       {
//         if(relations.at(i).groundTruth == relations.at(i).prediction) 
// 	{
//           if(relations.at(i).groundTruth == 1)        // true positive
//             tp++; 
//           else                                        // true negative
//             tn++;
//         }
//         else
// 	{
//           if(relations.at(i).groundTruth == 1)        // false positive
//             fn++; 
//           else                                        // false negative
//             fp++;
// 	}
//           
//         // now write output
//         double tpd = (double) tp;
//         double tnd = (double) tn;
//         double fpd = (double) fp;
//         double fnd = (double) fn;
//         std::fprintf(PPfileAnaNew,"%u, %u, %u, %u, %4.5f\n", tp, tn, fp, fn, 0.5*( (fpd/(tpd+fpd)) + (fnd/(tnd+fnd))));
//       }
//       
//       if(relations.at(i).type == 2 && relations.at(i).groundTruth != -1) 
//       {
//         if(relations.at(i).groundTruth == relations.at(i).prediction) 
// 	{
//           if(relations.at(i).groundTruth == 1)        // true positive
//             tp2++; 
//           else                                        // true negative
//             tn2++;
//         }
//         else
// 	{
//           if(relations.at(i).groundTruth == 1)        // false positive
//             fn2++; 
//           else                                        // false negative
//             fp2++;
// 	}
//           
//         // now write output
//         double tpd2 = (double) tp2;
//         double tnd2 = (double) tn2;
//         double fpd2 = (double) fp2;
//         double fnd2 = (double) fn2;
//         std::fprintf(PPfileAnaNew2,"%u, %u, %u, %u, %4.5f\n", tp2, tn2, fp2, fn2, 0.5*( (fpd2/(tpd2+fpd2)) + (fnd2/(tnd2+fnd2))));
//       }
//     }
//     std::fclose(PPfileAnaNew);
//     std::fclose(PPfileAnaNew2);
//     
//     /// Print testset results    
//     FILE *PPfile = std::fopen("./PP-Testset.txt", "a");         // structural level results
//     FILE *PP2file = std::fopen("./PP2-Testset.txt", "a");       // assembly level results
// 
//     for(int i=0; i<relations.size(); i++)
//     {
//       if(relations.at(i).groundTruth == 0  || relations.at(i).groundTruth == 1)   // write only relations with known groundTruth
//       {
//         if(relations.at(i).type == 1)
//         {
//           std::fprintf(PPfile,"%u ", relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++) {
//             if(relations.at(i).rel_value.at(j) != relations.at(i).rel_value.at(j))
//               printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
//             std::fprintf(PPfile,"%u:%6.5f ", j+1, relations.at(i).rel_value.at(j));
//           }
//           std::fprintf(PPfile,"\n");
//         }
//         else if(relations.at(i).type == 2)
//         {
//           fprintf(PP2file,"%u ", relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++) {
//             if(relations.at(i).rel_value.at(j) != relations.at(i).rel_value.at(j))
//               printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
//             fprintf(PP2file,"%u:%6.5f ", j+1, relations.at(i).rel_value.at(j));
//           }
//           fprintf(PP2file,"\n");
//         }
//         else
//           printf("[SVMFileCreator::process] Warning: Unknown type of relation received.\n");
//       }
//     }
//     std::fclose(PPfile);
//     std::fclose(PP2file);
// 
//     if(analyze)
//     {
//       static int counter = 0;
//       
//       FILE *PPfileAna = std::fopen("./PP-Testset.txt.ana", "a");                           // All relations (with unknown ones)
//       FILE *PPfilePos = std::fopen("./PP-Testset.txt.pos", "a");
//       FILE *PPfileNeg = std::fopen("./PP-Testset.txt.neg", "a");
//       FILE *PP2fileAna = std::fopen("./PP2-Testset.txt.ana", "a");
//       FILE *PP2filePos = std::fopen("./PP2-Testset.txt.pos", "a");
//       FILE *PP2fileNeg = std::fopen("./PP2-Testset.txt.neg", "a");
//       FILE *PPfileAnaRel = std::fopen("./PP-Testset.txt.rel.ana", "a");                    // All relations (with unknown ones)
//       FILE *PP2fileAnaRel = std::fopen("./PP2-Testset.txt.rel.ana", "a");                  // All relations (with unknown ones)
//       
//       for(int i=0; i<relations.size(); i++)
//       {
//         if(relations.at(i).type == 1)
//         {
//           std::fprintf(PPfileAna,"%u, ", relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++)
//             std::fprintf(PPfileAna,"%6.5f, ", relations.at(i).rel_value.at(j));
//           std::fprintf(PPfileAna,"\n");
//           
//           std::fprintf(PPfileAnaRel,"%u: [%u][%u] (%u), ", counter, relations.at(i).id_0, relations.at(i).id_1, relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++)
//             std::fprintf(PPfileAnaRel,"%6.5f, ", relations.at(i).rel_value.at(j));
//           std::fprintf(PPfileAnaRel,"\n");          
//           
//           if(relations.at(i).groundTruth == 1) {
//             for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PPfilePos,"%6.5f, ", relations.at(i).rel_value.at(j));
//             std::fprintf(PPfilePos,"\n");
//           }
//           else if(relations.at(i).groundTruth == 0) {
//             for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PPfileNeg,"%6.5f, ", relations.at(i).rel_value.at(j));
//             std::fprintf(PPfileNeg,"\n");
//           }
//         }
//         if(relations.at(i).type == 2)
//         {
//           std::fprintf(PP2fileAna,"%u, ", relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++)
//             std::fprintf(PP2fileAna,"%6.5f, ", relations.at(i).rel_value.at(j));
//           std::fprintf(PP2fileAna,"\n");
//   
//           std::fprintf(PP2fileAnaRel,"%u: [%u][%u] (%u), ", counter, relations.at(i).id_0, relations.at(i).id_1, relations.at(i).groundTruth);
//           for(int j=0; j<relations.at(i).rel_value.size(); j++)
//             std::fprintf(PP2fileAnaRel,"%6.5f, ", relations.at(i).rel_value.at(j));
//           std::fprintf(PP2fileAnaRel,"\n");   
//           
//           if(relations.at(i).groundTruth == 1) {
//             for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PP2filePos,"%6.5f, ", relations.at(i).rel_value.at(j));
//             std::fprintf(PP2filePos,"\n");
//           }
//           else if(relations.at(i).groundTruth == 0) {
//             for(int j=0; j<relations.at(i).rel_value.size(); j++)
//               std::fprintf(PP2fileNeg,"%6.5f, ", relations.at(i).rel_value.at(j));
//             std::fprintf(PP2fileNeg,"\n");
//           }
//         }
//       }
//       std::fclose(PPfileAna);
//       std::fclose(PPfilePos);
//       std::fclose(PPfileNeg);
//       std::fclose(PP2fileAna);
//       std::fclose(PP2filePos);
//       std::fclose(PP2fileNeg);
//       std::fclose(PPfileAnaRel);
//       std::fclose(PP2fileAnaRel);
//       counter++;
//     } 
//   } 
}

} 











