/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author thomas.moerwald
 *
 */

#include <stdexcept>
#include <wordexp.h>
#include <sstream>

#include "tgModelLoader.h"

using namespace TomGine;
using namespace std;

std::string
expandName (const std::string &fileString)
{
  std::stringstream ss;
  wordexp_t p;
  char** w;
  wordexp (fileString.c_str (), &p, 0);
  w = p.we_wordv;
  for (size_t i = 0; i < p.we_wordc; i++)
  {
    ss << w[i];
  }
  wordfree (&p);
  return ss.str ();
}

// *** PRIVATE ***

// Tests if property of file is available in list (data structure)
bool
tgModelLoader::propertyIsInList (PlyProperty* prop, PlyProperty* list, int n, int* index)
{

  for (int i = 0; i < n; i++)
  {
    if (equal_strings (prop->name, list[i].name))
    {
      *index = i;
      return true;
    }
  }
  return false;
}

// *** PUBLIC ***

// read ply file
bool
tgModelLoader::LoadPly (tgModel &model, const char* filename)
{
  PlyFile* plyfile;
  int i, j;
  int nelems;
  char **elist;
  int file_type;
  float version;
  char *elem_name;
  //PlyElement *elem_ptr;
  PlyProperty **plist;
  //PlyProperty* prop_ptr;
  int num_elems;
  int nprops;
  int index;
  char** obj_info;
  int num_obj_info;

  int num_vertices = 0;
  int num_faces = 0;
  std::vector<PlyVertex> plyvertexlist;
  std::vector<PlyFace> plyfacelist;
  std::vector<PlyEdge> plyedgelist;
  vector<string> texFilenames;
  string strFilename = expandName (filename);

  // **********************************************
  // Read data from ply file

  // open file
  plyfile = ply_open_for_reading (strFilename.c_str (), &nelems, &elist, &file_type, &version);
  if (plyfile == 0)
  {
    char errmsg[256];
    sprintf (errmsg, "[tgModelLoader::read] Cannot open ply file '%s'\n", strFilename.c_str ());
    throw runtime_error (errmsg);
  }
  // 	sprintf(model.m_modelname, "%s", filename);

  // Load texture files from obj_info (=texture-filename)
  obj_info = ply_get_obj_info (plyfile, &num_obj_info);
  if (num_obj_info < 1)
  {
    //printf("[tgModelLoader::read] Warning no texture found in model %s\n", filename);
  }
  else if (num_obj_info >= 1)
  {
    for (i = 0; i < num_obj_info; i++)
    {
      texFilenames.push_back ("");
      texFilenames[i].append (filename, 0, strFilename.find_last_of ("/") + 1);
      texFilenames[i].append (obj_info[i]);
      // 			printf("[tgModelLoader::read] obj_info: %s\n", texFilenames[i].c_str());
    }
  }

  // list of property information for a vertex
  PlyProperty vert_props[] = { {(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,x), 0, 0, 0, 0},
                               {(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,y), 0, 0, 0, 0},
                               {(char*)"z", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,z), 0, 0, 0, 0},
                               {(char*)"nx", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nx), 0, 0, 0, 0},
                               {(char*)"ny", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,ny), 0, 0, 0, 0},
                               {(char*)"nz", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nz), 0, 0, 0, 0},
                               {(char*)"u", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,s), 0, 0, 0, 0},
                               {(char*)"v", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,t), 0, 0, 0, 0},
                               {(char*)"red", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,r), 0, 0, 0, 0},
                               {(char*)"green", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,g), 0, 0, 0, 0},
                               {(char*)"blue", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,b), 0, 0, 0, 0}};

  // list of property information for a face
  PlyProperty face_props[] = { /* list of property information for a vertex */
                               {(char*)"vertex_indices", PLY_USHORT, PLY_UINT, offsetof(PlyFace,v), 1, PLY_USHORT, PLY_UINT,
                                offsetof(PlyFace,nverts)}, };

  // list of property information for an edge
  PlyProperty edge_props[] = { {(char*)"start", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,start), 0, 0, 0, 0},
                               {(char*)"end", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,end), 0, 0, 0, 0}};

  for (i = 0; i < nelems; i++)
  {
    // get description of element
    elem_name = elist[i];
    plist = ply_get_element_description (plyfile, elem_name, &num_elems, &nprops);

    // *** Read Vertices ***
    if (equal_strings ((char*)"vertex", elem_name))
    {
      // allocate memory for vertices
      num_vertices = num_elems;

      plyvertexlist.resize(num_vertices);

      // setup property specification for elements
      for (j = 0; j < nprops; j++)
      {
        if (propertyIsInList (plist[j], vert_props, 11, &index))
          ply_get_property (plyfile, elem_name, &vert_props[index]);
      }

      // grab all vertex elements
      for (j = 0; j < num_elems; j++)
      {
        ply_get_element (plyfile, &plyvertexlist[j]);
      }

    }

    // *** Read Faces ***
    if (equal_strings ((char*)"face", elem_name))
    {
      // allocate memory for faces
      num_faces = num_elems;
      plyfacelist.resize(num_faces);

      // setup property specification for elements
      for (j = 0; j < nprops && j < 1; j++)
      {
        ply_get_property (plyfile, elem_name, &face_props[j]);
      }

      // grab all face elements
      for (j = 0; j < num_elems; j++)
      {
        ply_get_element (plyfile, &plyfacelist[j]);
        //printf ("face: %d %d %d %d\n", m_facelist[j].v[0], m_facelist[j].v[1], m_facelist[j].v[2], m_facelist[j].v[3]);
      }
    }

    // *** Read Edges ***
    if (equal_strings ((char*)"edge", elem_name))
    {
      // allocate memory for edges
      plyedgelist.resize(num_elems);

      // setup property specification for elements
      for (j = 0; j < nprops && j < 2; j++)
      {
        ply_get_property (plyfile, elem_name, &edge_props[j]);
      }

      // grab all edge elements
      for (j = 0; j < num_elems; j++)
      {
        ply_get_element (plyfile, &plyedgelist[j]);
        //printf("edge: %d %d\n", m_edgelist[j].start, m_edgelist[j].end);
      }
    }
  }
  ply_close (plyfile);

  // **********************************************
  // Convert ply to model

  // Parse through vertex list
  for (i = 0; i < num_vertices; i++)
  {
    tgVertex v;
    v.pos.x = plyvertexlist[i].x;
    v.pos.y = plyvertexlist[i].y;
    v.pos.z = plyvertexlist[i].z;
    v.normal.x = plyvertexlist[i].nx;
    v.normal.y = plyvertexlist[i].ny;
    v.normal.z = plyvertexlist[i].nz;
    v.texCoord.x = plyvertexlist[i].s;
    v.texCoord.y = plyvertexlist[i].t;
    v.color[0] = plyvertexlist[i].r;
    v.color[1] = plyvertexlist[i].g;
    v.color[2] = plyvertexlist[i].b;
    model.m_vertices.push_back (v);

    // 		printf("v: %f %f %f\n", v.pos.x, v.pos.y, v.pos.z);
  }

  // Parse through face list
  for (i = 0; i < num_faces; i++)
  {
    tgFace f;
    for (j = 0; j < plyfacelist[i].nverts; j++)
    {
      f.v.push_back (plyfacelist[i].v[j]);
    }
    model.m_faces.push_back (f);
  }

  // 	model.ComputeFaceNormals();

  return true;
}

bool
tgModelLoader::SavePly (const tgModel &model, const std::string &filename,
                        bool with_normals, bool with_tex_coords, bool with_color)
{
  FILE* pFile = 0;

  pFile = fopen (filename.c_str (), "w");

  if (!pFile)
  {
    char errmsg[128];
    sprintf (errmsg, "[tgModelLoader::SavePly] Can not create file '%s'! Path existing?", filename.c_str ());
    throw std::runtime_error (errmsg);
  }

  // ****************************************************
  // Header
  fputs ("ply\n", pFile);
  fputs ("format ascii 1.0\n", pFile);

  // Header of vertex
  fputs ("element vertex ", pFile);
  fprintf (pFile, "%d\n", (int)model.m_vertices.size ());
  fputs ("property float x\n", pFile);
  fputs ("property float y\n", pFile);
  fputs ("property float z\n", pFile);
  if(with_normals)
  {
    fputs ("property float nx\n", pFile);
    fputs ("property float ny\n", pFile);
    fputs ("property float nz\n", pFile);
  }
  if(with_tex_coords)
  {
    fputs ("property float u\n", pFile);
    fputs ("property float v\n", pFile);
  }
  if(with_color)
  {
    fputs ("property uchar red\n", pFile);
    fputs ("property uchar green\n", pFile);
    fputs ("property uchar blue\n", pFile);
    fputs ("property uchar alpha\n", pFile);
  }

  // Header of face
  fputs ("element face ", pFile);
  fprintf (pFile, "%d\n", (int)model.m_faces.size ());
  fputs ("property list uchar uint vertex_indices\n", pFile);

  fputs ("end_header\n", pFile);

  // Data of vertex
  for (size_t i = 0; i < model.m_vertices.size (); i++)
  {
    const tgVertex &v = model.m_vertices[i];

    fprintf (pFile, "%f %f %f", v.pos.x, v.pos.y, v.pos.z);

    if(with_normals)
      fprintf (pFile, " %f %f %f", v.normal.x, v.normal.y, v.normal.z);

    if(with_tex_coords)
      fprintf (pFile, " %f %f", v.texCoord.x, v.texCoord.y);

    if(with_color)
      fprintf (pFile, " %d %d %d %d", v.color[0], v.color[1], v.color[2], v.color[3]);

    fprintf(pFile, "\n");
  }

  // Data of face
  for (size_t i = 0; i < model.m_faces.size (); i++)
  {
    const tgFace &f = model.m_faces[i];
    fprintf (pFile, "%d", (int)f.v.size ());
    for (size_t j = 0; j < f.v.size (); j++)
      fprintf (pFile, " %d", f.v[j]);
    fputs ("\n", pFile);
  }

  fclose (pFile);

  printf ("saved: '%s'\n", filename.c_str ());

  return true;
}

