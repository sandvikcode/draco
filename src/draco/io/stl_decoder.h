// Copyright 2018 Daniel Wedlund.
//

#ifndef DRACO_IO_STL_DECODER_H_
#define DRACO_IO_STL_DECODER_H_

#include <string>
#include <unordered_map>

#include "draco/draco_features.h"

#include "draco/core/decoder_buffer.h"
#include "draco/core/status.h"
#include "draco/mesh/mesh.h"

namespace draco {

// Decodes a STL file into draco::Mesh (or draco::PointCloud if the
// connectivity data is not needed).. This decoder can handle decoding of
// positions and triangular faces.
// All other geometry properties are ignored.
class StlDecoder {
 public:
  StlDecoder();

  // Decodes an stl file stored in the input file.
  // Returns nullptr if the decoding failed.
  Status DecodeFromFile(const std::string &file_name, Mesh *out_mesh);
  Status DecodeFromFile(const std::string &file_name,
                        PointCloud *out_point_cloud);

  Status DecodeFromBuffer(DecoderBuffer *buffer, Mesh *out_mesh);
  Status DecodeFromBuffer(DecoderBuffer *buffer, PointCloud *out_point_cloud);

  // Flag that can be used to turn on/off deduplication of input values.
  // This should be disabled only when we are sure that the input data does not
  // contain any duplicate entries.
  // Default: true
  void set_deduplicate_input_values(bool v) { deduplicate_input_values_ = v; }
  // Flag for whether using metadata to record other information in the stl
  // file, e.g. material names, object names.
  void set_use_metadata(bool flag) { use_metadata_ = flag; }

 protected:
  Status DecodeInternal();
  DecoderBuffer *buffer() { return &buffer_; }

 private:
  // Resets internal counters for attributes and faces.
  void ResetCounters();

  // Parses the next mesh property definition (position, tex coord, normal, or
  // face). If the parsed data is unrecognized, it will be skipped.
  // Returns false when the end of file was reached.
  bool ParseDefinition(Status *status);

  // Attempts to parse definition of position, normal, tex coord, or face
  // respectively.
  // Returns false when the parsed data didn't contain the given definition.
  bool ParseHeader(Status *status);
  bool ParseTriangleCount(Status *status);
  bool ParseTriangles(Status *status);

  // Maps specified point index to the parsed vertex indices (triplet of
  // position, texture coordinate, and normal indices) .
  void MapPointToVertexIndices(PointIndex pi,
                               const std::array<int32_t, 3> &indices);


  // If set to true, the parser will count the number of various definitions
  // but it will not parse the actual data or add any new entries to the mesh.
  bool counting_mode_;
  int num_obj_faces_;
  int num_positions_;
  int num_tex_coords_;
  int num_normals_;
  int num_materials_;
  int last_sub_obj_id_;

  int pos_att_id_;
  int tex_att_id_;
  int norm_att_id_;
  int material_att_id_;
  int sub_obj_att_id_;  // Attribute id for storing sub-objects.

  bool deduplicate_input_values_;

  int last_material_id_;
  std::string material_file_name_;

  std::string input_file_name_;

  std::unordered_map<std::string, int> material_name_to_id_;
  std::unordered_map<std::string, int> obj_name_to_id_;

  bool use_metadata_;

  DecoderBuffer buffer_;

  // Data structure that stores the decoded data. |out_point_cloud_| must be
  // always set but |out_mesh_| is optional.
  Mesh *out_mesh_;
  PointCloud *out_point_cloud_;
};

}  // namespace draco

#endif  // DRACO_IO_STL_DECODER_H_
