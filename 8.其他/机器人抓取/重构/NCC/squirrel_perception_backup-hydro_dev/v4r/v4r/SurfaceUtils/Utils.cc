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

#include "Utils.hh"

#include <pcl/sample_consensus/model_types.h>

namespace surface
{

  using namespace std;

  /********************** Utils ************************
   * Constructor/Destructor
   */
  Utils::Utils ()
  {
  }

  Utils::~Utils ()
  {
  }

  /************************** PRIVATE ************************/

  /************************** PUBLIC *************************/

  /**
   * ConvertCloud2Image
   */
  void
  Utils::ConvertCloud2Image (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat_<cv::Vec3b> &image)
  {
    unsigned pcWidth = cloud.width;
    unsigned pcHeight = cloud.height;
    RGBValue color;

    image = cv::Mat_<cv::Vec3b> (pcHeight, pcWidth);

    for (unsigned row = 0; row < pcHeight; row++)
    {
      for (unsigned col = 0; col < pcWidth; col++)
      {
        cv::Vec3b &cvp = image (row, col);
        const pcl::PointXYZRGB &pt = cloud (col, row);

        if (pt.rgb != pt.rgb)
        {
          cvp[0] = 0;
          cvp[1] = 0;
          cvp[2] = 0;
        }
        else
        {
          color.float_value = pt.rgb;
          cvp[2] = color.r;
          cvp[1] = color.g;
          cvp[0] = color.b;
        }
      }
    }
  }

  bool
  Utils::convertSurfaces2Brep (const std::vector<surface::SurfaceModel::Ptr> &surfaces, ON_Brep &brep)
  {
    for (size_t i = 0; i < surfaces.size (); i++)
    {
      surface::SurfaceModel::Ptr surface = surfaces[i];

      ON_NurbsSurface* ns = new ON_NurbsSurface (surface->nurbs);
      int si = brep.AddSurface (ns);
      ON_BrepFace &face = brep.NewFace (si);

      if (!surface->curves_param.empty ())
      {
        ON_BrepLoop &loop = brep.NewLoop (ON_BrepLoop::outer, face);
        for (size_t i = 0; i < surface->curves_param.size (); i++)
        {
          ON_NurbsCurve* nc = new ON_NurbsCurve (surface->curves_param[i]);
          int c2i = brep.AddTrimCurve (nc);
          ON_BrepTrim &trim = brep.NewTrim (c2i);
          loop.m_ti.Append (trim.m_trim_index);
        }
      }
    }
    return true;
  }

  bool
  Utils::convertSurfaces2Brep (const std::vector<surface::SurfaceModel::Ptr> &surfaces,
                               const pcl::PointCloud<pcl::PointXYZRGB> &cloud, ON_Brep &brep)
  {
    for (size_t i = 0; i < surfaces.size (); i++)
    {
      surface::SurfaceModel::Ptr surface = surfaces[i];

      ON_NurbsSurface* ns = new ON_NurbsSurface (surface->nurbs);
      int si = brep.AddSurface (ns);
      ON_BrepFace &face = brep.NewFace (si);

      if (!surface->curves_param.empty ())
      {
        ON_BrepLoop &loop = brep.NewLoop (ON_BrepLoop::outer, face);
        for (size_t j = 0; j < surface->curves_param.size (); j++)
        {
          ON_NurbsCurve* nc = new ON_NurbsCurve (surface->curves_param[j]);
          int c2i = brep.AddTrimCurve (nc);
          ON_BrepTrim &trim = brep.NewTrim (c2i);
          loop.m_ti.Append (trim.m_trim_index);
        }
      }

      if (surface->indices.size () == surface->normals.size ())
      {

        for (size_t j = 0; j < surface->indices.size (); j++)
        {
          const pcl::PointXYZRGB &p = cloud.at (surface->indices[j]);
          Eigen::Vector3d &n = surface->normals[j];

          if (!std::isnan (p.z) && std::isfinite (p.z))
          {
            ON_BrepVertex bv (i); // set index of surface this point belongs to
            bv.SetPoint (ON_3dPoint (p.x, p.y, p.z));
            brep.m_V.Append (bv);

            ON_BrepVertex bn (i);
            bn.SetPoint (ON_3dPoint (n (0), n (1), n (2)));
            brep.m_V.Append (bn);
          }
        }

      }
      else
      {
        printf ("[Utils::convertSurfaces2Brep] WARNING: indices- and normals-size do not match\n");
      }

    }
    return true;
  }

  bool
  Utils::convertBrep2Surfaces (const ON_Brep &brep, std::vector<surface::SurfaceModel::Ptr> &surfaces)
  {
    for (int fi = 0; fi < brep.m_F.Count (); fi++)
    {
      const ON_BrepFace& face = brep.m_F[fi];

      // pSrf = underlying untrimmed surface
      const ON_Surface* pSrf = NULL;
      if (face.m_si < 0 || face.m_si >= brep.m_S.Count ())
      {
        printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_F[%d].m_si\n", fi);
        return false;
      }
      else
      {
        pSrf = brep.m_S[face.m_si];
        if (!pSrf)
        {
          printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_S[%d] is NULL\n", face.m_si);
          return false;
        }
      }

      surface::SurfaceModel::Ptr surface (new surface::SurfaceModel);

      if (pSrf->ClassId () == ON_NurbsSurface ().ClassId ())
      {
        ON_NurbsSurface* pNurbsSurface = (ON_NurbsSurface*)pSrf;
        //      pNurbsSurface->Dump (out);

        surface->nurbs = *pNurbsSurface;
        if (surface->nurbs.Order (0) == 2)
          surface->type = pcl::SACMODEL_PLANE;
        else
          surface->type = MODEL_NURBS;
      }
      else
      {
        printf ("[FileSystem::convertBrep2Surfaces] ERROR: Surface class not supported: %s\n", pSrf->ClassId ()->ClassName ());
        return false;
      }

      // The face is trimmed with one or more trimming loops.
      //
      // All the 2d trimming curves are oriented so that the
      // active region of the trimmed surface lies to the left
      // of the 2d trimming curve.
      //
      // If face.m_bRev is true, the orientations of the face in
      // the b-rep is opposited the natural parameteric orientation
      // of the surface.

      // loop_count = number of trimming loops on this face (>=1)
      const int loop_count = face.m_li.Count ();

      int fli; // face's loop index
      for (fli = 0; fli < loop_count; fli++)
      {
        const int li = face.m_li[fli]; // li = brep loop index
        const ON_BrepLoop& loop = brep.m_L[li];

        // loop_edge_count = number of trimming edges in this loop
        const int loop_trim_count = loop.m_ti.Count ();

        int lti; // loop's trim index
        for (lti = 0; lti < loop_trim_count; lti++)
        {
          const int ti = loop.m_ti[lti]; // ti = brep trim index
          const ON_BrepTrim& trim = brep.m_T[ti];

          //////////////////////////////////////////////////////
          // 2d trimming information
          //
          // Each trim has a 2d parameter space curve.
          const ON_Curve* p2dCurve = NULL;
          const int c2i = trim.m_c2i; // c2i = brep 2d curve index
          if (c2i < 0 || c2i >= brep.m_C2.Count ())
          {
            printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_T[%d].m_c2i\n", ti);
            return false;
          }
          else
          {
            p2dCurve = brep.m_C2[c2i];
            if (!p2dCurve)
              printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_C2[%d] is NULL\n", c2i);
          }

          if (p2dCurve->ClassId () == ON_NurbsCurve ().ClassId ())
          {
            ON_NurbsCurve* pNurbsCurve = (ON_NurbsCurve*)p2dCurve;
            surface->curves_param.push_back (*pNurbsCurve);
            //          pNurbsCurve->Dump (out);
          }
          else
          {
            printf ("[FileSystem::convertBrep2Surfaces] ERROR: trimming curve not supported: %s\n",
                    p2dCurve->ClassId ()->ClassName ());
            return false;
          }

        }
      }

      surface->idx = fi;
      surfaces.push_back (surface);
    }

    return true;
  }

  bool
  Utils::convertBrep2Surfaces (const ON_Brep &brep, std::vector<surface::SurfaceModel::Ptr> &surfaces,
                               pcl::PointCloud<pcl::PointXYZRGB> &cloud)
  {
    for (int fi = 0; fi < brep.m_F.Count (); fi++)
    {
      const ON_BrepFace& face = brep.m_F[fi];

      // pSrf = underlying untrimmed surface
      const ON_Surface* pSrf = NULL;
      if (face.m_si < 0 || face.m_si >= brep.m_S.Count ())
      {
        printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_F[%d].m_si\n", fi);
        return false;
      }
      else
      {
        pSrf = brep.m_S[face.m_si];
        if (!pSrf)
        {
          printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_S[%d] is NULL\n", face.m_si);
          return false;
        }
      }

      surface::SurfaceModel::Ptr surface (new surface::SurfaceModel);

      if (pSrf->ClassId () == ON_NurbsSurface ().ClassId ())
      {
        ON_NurbsSurface* pNurbsSurface = (ON_NurbsSurface*)pSrf;
        //      pNurbsSurface->Dump (out);

        surface->nurbs = *pNurbsSurface;
        if (surface->nurbs.Order (0) == 2)
          surface->type = pcl::SACMODEL_PLANE;
        else
          surface->type = MODEL_NURBS;
      }
      else
      {
        printf ("[FileSystem::convertBrep2Surfaces] ERROR: Surface class not supported: %s\n", pSrf->ClassId ()->ClassName ());
        return false;
      }

      // The face is trimmed with one or more trimming loops.
      //
      // All the 2d trimming curves are oriented so that the
      // active region of the trimmed surface lies to the left
      // of the 2d trimming curve.
      //
      // If face.m_bRev is true, the orientations of the face in
      // the b-rep is opposited the natural parameteric orientation
      // of the surface.

      // loop_count = number of trimming loops on this face (>=1)
      const int loop_count = face.m_li.Count ();

      int fli; // face's loop index
      for (fli = 0; fli < loop_count; fli++)
      {
        const int li = face.m_li[fli]; // li = brep loop index
        const ON_BrepLoop& loop = brep.m_L[li];

        // loop_edge_count = number of trimming edges in this loop
        const int loop_trim_count = loop.m_ti.Count ();

        int lti; // loop's trim index
        for (lti = 0; lti < loop_trim_count; lti++)
        {
          const int ti = loop.m_ti[lti]; // ti = brep trim index
          const ON_BrepTrim& trim = brep.m_T[ti];

          //////////////////////////////////////////////////////
          // 2d trimming information
          //
          // Each trim has a 2d parameter space curve.
          const ON_Curve* p2dCurve = NULL;
          const int c2i = trim.m_c2i; // c2i = brep 2d curve index
          if (c2i < 0 || c2i >= brep.m_C2.Count ())
          {
            printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_T[%d].m_c2i\n", ti);
            return false;
          }
          else
          {
            p2dCurve = brep.m_C2[c2i];
            if (!p2dCurve)
              printf ("[FileSystem::convertBrep2Surfaces] ERROR: invalid brep.m_C2[%d] is NULL\n", c2i);
          }

          if (p2dCurve->ClassId () == ON_NurbsCurve ().ClassId ())
          {
            ON_NurbsCurve* pNurbsCurve = (ON_NurbsCurve*)p2dCurve;
            surface->curves_param.push_back (*pNurbsCurve);
            //          pNurbsCurve->Dump (out);
          }
          else
          {
            printf ("[FileSystem::convertBrep2Surfaces] ERROR: trimming curve not supported: %s\n",
                    p2dCurve->ClassId ()->ClassName ());
            return false;
          }

        }
      }

      surface->idx = fi;
      surfaces.push_back (surface);
    }

    cloud.clear ();
    for (int i = 0; i < brep.m_V.Count () - 1; i += 2)
    {
      const ON_BrepVertex &bv = brep.m_V[i];
      const ON_BrepVertex &bn = brep.m_V[i + 1];

      if (bv.m_vertex_index < 0 || bv.m_vertex_index >= (int)surfaces.size ())
      {
        printf ("[surface::Utils::convertBrep2Surfaces] vertex index out of surface bounds\n");
        return false;
      }

      if (bn.m_vertex_index != bv.m_vertex_index)
      {
        printf ("[surface::Utils::convertBrep2Surfaces] vertex index does not match normal index\n");
        return false;
      }

      surfaces[bv.m_vertex_index]->indices.push_back (cloud.size ());
      surfaces[bn.m_vertex_index]->normals.push_back (Eigen::Vector3d (bn.point.x, bn.point.y, bn.point.z));

      pcl::PointXYZRGB p;
      p.x = bv.point.x;
      p.y = bv.point.y;
      p.z = bv.point.z;

      cloud.push_back (p);
    }

    return true;
  }

} //-- THE END --

