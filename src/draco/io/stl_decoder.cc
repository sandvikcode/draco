// Copyright 2018 Daniel Wedlund.
//

#include "draco/io/stl_decoder.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <string>

#include "draco/io/parser_utils.h"
#include "draco/metadata/geometry_metadata.h"

namespace draco
{

StlDecoder::StlDecoder()
    : counting_mode_(true),
      num_obj_faces_(0),
      num_positions_(0),
      num_tex_coords_(0),
      num_normals_(0),
      num_materials_(0),
      last_sub_obj_id_(0),
      pos_att_id_(-1),
      tex_att_id_(-1),
      norm_att_id_(-1),
      material_att_id_(-1),
      sub_obj_att_id_(-1),
      deduplicate_input_values_(true),
      last_material_id_(0),
      use_metadata_(false),
      out_mesh_(nullptr),
      out_point_cloud_(nullptr) {}

Status StlDecoder::DecodeFromFile(const std::string &file_name,
                                  Mesh *out_mesh)
{
  out_mesh_ = out_mesh;
  return DecodeFromFile(file_name, static_cast<PointCloud *>(out_mesh));
}

Status StlDecoder::DecodeFromFile(const std::string &file_name,
                                  PointCloud *out_point_cloud)
{
  std::ifstream file(file_name, std::ios::binary);
  if (!file)
    return Status(Status::IO_ERROR);
  // Read the whole file into a buffer.
  auto pos0 = file.tellg();
  file.seekg(0, std::ios::end);
  auto file_size = file.tellg() - pos0;
  if (file_size == 0)
    return Status(Status::IO_ERROR);
  file.seekg(0, std::ios::beg);
  std::vector<char> data(file_size);
  file.read(&data[0], file_size);
  buffer_.Init(&data[0], file_size);

  out_point_cloud_ = out_point_cloud;
  input_file_name_ = file_name;
  return DecodeInternal();
}

Status StlDecoder::DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh)
{
  out_mesh_ = out_mesh;
  return DecodeFromBuffer(buffer, static_cast<PointCloud *>(out_mesh));
}

Status StlDecoder::DecodeFromBuffer(DecoderBuffer *buffer,
                                    PointCloud *out_point_cloud)
{
  out_point_cloud_ = out_point_cloud;
  buffer_.Init(buffer->data_head(), buffer->remaining_size());
  return DecodeInternal();
}

Status StlDecoder::DecodeInternal()
{
  // In the first pass, count the number of different elements in the geometry.
  // In case the desired output is just a point cloud (i.e., when
  // out_mesh_ == nullptr) the decoder will ignore all information about the
  // connectivity that may be included in the source data.
  counting_mode_ = true;
  ResetCounters();
  material_name_to_id_.clear();
  last_sub_obj_id_ = 0;
  // Parse all lines.
  Status status(Status::OK);

  if (!ParseHeader(&status))
  {
    return status;
  }
  if (!ParseTriangleCount(&status))
  {
    return status;
  }
  if (!ParseTriangles(&status))
  {
    return status;
  }

  if (!status.ok())
    return status;

  bool use_identity_mapping = false;

  // Initialize point cloud and mesh properties.
  if (out_mesh_)
  {
    out_point_cloud_->set_num_points(3 * num_obj_faces_);

    // Add attributes if they are present in the input data.
    if (num_positions_ > 0)
    {
      GeometryAttribute va;
      va.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false,
              sizeof(float) * 3, 0);
      pos_att_id_ = out_point_cloud_->AddAttribute(va, use_identity_mapping,
                                                   num_positions_);
    }

    if (num_normals_ > 0)
    {
      GeometryAttribute va;
      va.Init(GeometryAttribute::NORMAL, nullptr, 3, DT_FLOAT32, false,
              sizeof(float) * 3, 0);
      norm_att_id_ =
          out_point_cloud_->AddAttribute(va, use_identity_mapping, num_obj_faces_);
    }

    // Extra data - not supported
    /*
             if (num_materials_ > 0 && num_obj_faces_ > 0) {
             GeometryAttribute va;
             va.Init(GeometryAttribute::GENERIC, nullptr, 1, DT_UINT16, false, 2, 0);
             extra_att_id_ =
             out_point_cloud_->AddAttribute(va, false, num_faces_);
             }
             */
  }

  // Now for the real deal
  counting_mode_ = false;
  ResetCounters();
  // Start parsing from the beginning of the buffer again.
  buffer()->StartDecodingFrom(0);

  if (!ParseHeader(&status))
  {
    return status;
  }
  if (!ParseTriangleCount(&status))
  {
    return status;
  }
  if (!ParseTriangles(&status))
  {
    return status;
  }

  if (!status.ok())
    return status;

  if (out_mesh_)
  {
    out_mesh_->SetNumFaces(num_obj_faces_);
    // Add faces with identity mapping between vertex and corner indices.
    // Duplicate vertices will get removed later.
    Mesh::Face face;
    for (FaceIndex i(0); i < num_obj_faces_; ++i)
    {
      for (int c = 0; c < 3; ++c)
        face[c] = 3 * i.value() + c;
      out_mesh_->SetFace(i, face);
    }
  }

#ifdef DRACO_ATTRIBUTE_DEDUPLICATION_SUPPORTED
  if (deduplicate_input_values_)
  {
    out_point_cloud_->DeduplicateAttributeValues();
  }
  out_point_cloud_->DeduplicatePointIds();
#endif
  return status;
}

void StlDecoder::ResetCounters()
{
  num_obj_faces_ = 0;
  num_positions_ = 0;
  num_tex_coords_ = 0;
  num_normals_ = 0;
  last_material_id_ = 0;
  last_sub_obj_id_ = 0;
}

bool StlDecoder::ParseHeader(Status *status)
{
  // Skip the header... might read meta-data in the future
  std::array<char, 80> c;
  if (!buffer()->Peek(&c))
  {
    *status = Status(Status::ERROR, "Failed to parse a 80B header");
    return false;
  }
  buffer()->Advance(80);
  return true;
}

bool StlDecoder::ParseTriangleCount(Status *status)
{
  uint32_t numtri;
  if (!buffer()->Decode(&numtri))
  {
    *status = Status(Status::ERROR, "Failed to parse a 4B triangle count");
    return false;
  }
  // We do not use the parsed triangle count since it can be wrong
  return true;
}

bool StlDecoder::ParseTriangles(Status *status)
{
  if (counting_mode_)
  {
    while (buffer()->remaining_size() >= 50)
    {
      //++num_normals_;
      ++num_obj_faces_;
      num_positions_ += 3;
      buffer()->Advance(50);
    }
    return true;
  }

  uint16_t attribute_count;
  float val[3];
  while (buffer()->remaining_size() >= 50)
  {
    // Read the normal of the facet
    buffer()->Decode(&val);
    /* 
            out_point_cloud_->attribute(norm_att_id_)
              ->SetAttributeValue(AttributeValueIndex(num_normals_), val);
            ++num_normals_;
            */

    // Read the three vertices
    for (int i = 0; i < 3; i++)
    {
      buffer()->Decode(&val);
      out_point_cloud_->attribute(pos_att_id_)
          ->SetAttributeValue(AttributeValueIndex(num_positions_), val);
      ++num_positions_;
    }

    for (int i = 0; i < 3; ++i)
    {
      const PointIndex vert_id(3 * num_obj_faces_ + i);
      out_point_cloud_->attribute(pos_att_id_)
          ->SetPointMapEntry(vert_id, AttributeValueIndex(3 * num_obj_faces_ + i));

      /*
      out_point_cloud_->attribute(norm_att_id_)
          ->SetPointMapEntry(vert_id, AttributeValueIndex(indices[2] - 1));
      */
    }
    ++num_obj_faces_;

    buffer()->Decode(&attribute_count);
    // TODO: Store the meta-data also...
    // Check if attribute_count == 0 ??
  }
  return true;
}

void StlDecoder::MapPointToVertexIndices(
    PointIndex vert_id, const std::array<int32_t, 3> &indices)
{

  out_point_cloud_->attribute(pos_att_id_)
      ->SetPointMapEntry(vert_id, AttributeValueIndex(indices[0] - 1));

  if (norm_att_id_ >= 0)
  {
    if (indices[2] > 0)
    {
      out_point_cloud_->attribute(norm_att_id_)
          ->SetPointMapEntry(vert_id, AttributeValueIndex(indices[2] - 1));
    }
    else if (indices[2] < 0)
    {
      out_point_cloud_->attribute(norm_att_id_)
          ->SetPointMapEntry(vert_id,
                             AttributeValueIndex(num_normals_ + indices[2]));
    }
    else
    {
      // Normal index not provided but expected. Insert 0 entry as the default
      // value.
      out_point_cloud_->attribute(norm_att_id_)
          ->SetPointMapEntry(vert_id, AttributeValueIndex(0));
    }
  }
}

} // namespace draco
