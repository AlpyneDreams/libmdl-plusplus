#pragma once

#include <span>
#include <cstdint>

//-----------------------------------------------------------------------------
namespace libmdl {

constexpr int MAX_NUM_LODS = 8;
constexpr int MAX_NUM_BONES_PER_VERT = 3;

using u8  = std::uint8_t;   static_assert(sizeof(u8)  == 1);
using i16 = std::int16_t;   static_assert(sizeof(i16) == 2);
using u16 = std::uint16_t;  static_assert(sizeof(u16) == 2);
using i32 = std::int32_t;   static_assert(sizeof(i32) == 4);
using u64 = std::uint64_t;  static_assert(sizeof(u64) == 8);
using b8  = bool;           static_assert(sizeof(b8)  == 1);
using f32 = float;          static_assert(sizeof(f32) == 4); // std::float32_t in C++23

//-----------------------------------------------------------------------------

struct ID32 {
  const i32 value;
  constexpr ID32(const char id[4])
    : value((id[0] << 0) | (id[1] << 8) | (id[2] << 16) | (id[3] << 24)) {}
};

//-----------------------------------------------------------------------------

#ifndef LIBMDL_CUSTOM_VECTORS
struct vec2 { f32 x, y; };        // Vector2D
struct vec3 { f32 x, y, z; };     // Vector
struct vec4 { f32 x, y, z, w; };  // Vector4D
#endif
struct AABB { vec3 min, max; };   // AABB_t

//-----------------------------------------------------------------------------

// Equivalent to CResourcePointer
template <typename T>
struct ResPtr {
  i32 offset;

  const T* get(const auto* base, size_t i = 0) const {
    const u8* start = reinterpret_cast<const u8*>(base) + offset;
    return reinterpret_cast<const T*>(start) + i;
  }

  // No accidentally passing pointers to pointers
  const T* get(const auto** base, size_t i = 0) const = delete;

  const T* operator()(const auto& base) const {
    return get(&base, 0);
  }
};

// Equivalent to CResourceString
struct ResString : public ResPtr<char> {};

struct ResStringView : public ResString {
  i32 length;
};

// Equivalent to CResourceArray
template <typename T>
struct ResArray {
  i32 count;
  i32 offset;

  const T* get(const void* base, size_t i) const {
    return reinterpret_cast<const T*>(reinterpret_cast<const u8*>(base) + offset) + i;
  }

  // No accidentally passing pointers to pointers
  const T* get(const auto** base, size_t i = 0) const = delete;

  const std::span<const T> operator()(const auto& base) const {
    return { get(&base, 0), size_t(count) };
  }
};

template <typename T>
struct NamedResArray : public ResArray<T> {
  i32 nameOffset;
};

//-----------------------------------------------------------------------------
// .MDL File Format (derived from public/studio.h)
// Contains overall model structure info
//-----------------------------------------------------------------------------
namespace mdl {
  static constexpr ID32 ID      = "IDST";
  static constexpr i32  VERSION = 49;

  // mstudioflex_t
  struct Flex {
    i32 flexDesc;
    f32 targets[4];
    i32 numVerts;
    i32 vertIndex;
    i32 flexPair;
    u8  vertAnimType;
    u8  unusedBytes[3];
    i32 unused[6];
  };

  // mstudio_modelvertexdata_t
  struct ModelVertexData {
    u64 pVertexData;
    u64 pTangentData;
    u64 pExtraData;
  };

#pragma pack( push, 4 )

  // mstudio_meshvertexdata_t
  struct MeshVertexData {
    i32 unused_modelVertexData;
    i32 numLODVertices[MAX_NUM_LODS];
    u64 modelVertexData;
  };

  // mstudiomesh_t
  struct Mesh {
    i32 material;
    ResPtr<struct Model> model;
    i32 numVertices;
    i32 vertexOffset;
    ResArray<Flex> flexes;
    i32 materialType;
    i32 materialParam;
    i32 id;
    vec3 center;
    MeshVertexData vertexData;
    i32 unused[6];
  };

  // mstudiomodel_t
  struct Model {
    char name[64];
    i32 type;
    f32 boundingRadius;
    ResArray<Mesh> meshes;
    i32 numVertices;
    i32 vertexIndex;
    i32 tangentsIndex;
    i32 numAttachments;
    i32 attachmentIndex;
    ResArray<struct Eyeball> eyeballs;
    ModelVertexData vertexData;
    i32 unused[4];
  };

#pragma pack( pop )

  // mstudioeyeball_t
  struct Eyeball {
    ResString name;
    i32 bone;
    vec3 org;
    f32 zOffset;
    f32 radius;
    vec3 up;
    vec3 forward;
    i32 texture;
    i32 unused1;
    f32 irisScale;
    i32 unused2;
    i32 upperFlexDesc[3];
    i32 lowerFlexDesc[3];
    f32 upperTarget[3];
    f32 lowerTarget[3];
    i32 upperLidFlexDesc;
    i32 lowerLidFlexDesc;
    i32 unused[4];
    b8  nonFACS;
    u8  unused3[3];
    i32 unused4[7];
  };

  // mstudiobodyparts_t
  struct BodyPart {
    ResString name;
    i32 numModels;
    i32 base;
    ResPtr<Model> modelData;
  };

  // mstudiotexture_t
  struct Texture {
    ResString name;
    i32 flags;
    i32 used;
    i32 unused1;

    mutable u64 material;
    mutable u64 clientMaterial;
    i32         unused[8];
  };

  // TODO: Fill in these structs
  struct Bone {};             // mstudiobone_t
  struct BoneController {};   // mstudiobonecontroller_t
  struct BoneFlexDriver {};   // mstudioboneflexdriver_t
  struct HitboxSet {};        // mstudiohitboxset_t
  struct AnimDesc {};         // mstudioanimdesc_t
  struct SeqDesc {};          // mstudioseqdesc_t
  struct Attachment {};       // mstudioattachment_t
  struct FlexDesc {};         // mstudioflexdesc_t
  struct FlexController {};   // mstudioflexcontroller_t
  struct FlexControllerUI {}; // mstudioflexcontrollerui_t
  struct FlexRule {};         // mstudioflexrule_t
  struct IKChain {};          // mstudioikchain_t
  struct IKLock {};           // mstudioiklock_t
  struct Mouth {};            // mstudiomouth_t
  struct PoseParamDesc {};    // mstudioposeparamdesc_t
  struct ModelGroup {};       // mstudiomodelgroup_t
  struct AnimBlock {};        // mstudioanimblock_t
  struct SrcBoneTransform {}; // mstudiosrcbonetransform_t
  struct LinearBone {};       // mstudiolinearbone_t
  struct BodyGroupPreset {};  // mstudiobodygrouppreset_t
  struct PhysFeModelDesc {};  // PhysFeModelDesc_t

  // studiohdr2_t
  struct Header2 {
    // Some resource offsets are relative to Header, not Header2
    ResArray<SrcBoneTransform>  srcBoneTransforms;
    i32                         illumPositionAttachmentIndex;
    f32                         maxEyeDeflection;
    ResPtr<LinearBone>          linearBones;
    ResString                   name;
    ResArray<BoneFlexDriver>    boneFlexDrivers;
    ResPtr<PhysFeModelDesc>     feModel;
    ResArray<BodyGroupPreset>   bodyGroupPresets;
    i32                         unused;
    // Additional data... (virtualModel, animblockModel, pVertexBase, pIndexBase, etc.)
  };

  // studiohdr_t
  struct Header {
    i32  id;
    i32  version;
    i32  checksum;
    char name[64];
    i32  length;
    vec3 eyePosition;
    vec3 illumPosition;
    AABB hull;
    AABB viewBox;
    i32  flags;
    ResArray<Bone>              bones;
    ResArray<BoneController>    boneControllers;
    ResArray<HitboxSet>         hitboxSets;
    ResArray<AnimDesc>          localAnims;
    ResArray<SeqDesc>           localSeqs;
    mutable i32                 activityListVersion;
    mutable i32                 eventsIndexed;
    ResArray<Texture>           textures;
    ResArray<ResString>         textureDirs;
    i32                         numSkinRef; // ?
    ResArray<i16>               skinRefs;
    ResArray<BodyPart>          bodyParts;
    ResArray<Attachment>        localAttachments;
    NamedResArray<u8>           localNodes;
    ResArray<FlexDesc>          flexDescs;
    ResArray<FlexController>    flexControllers;
    ResArray<FlexRule>          flexRules;
    ResArray<IKChain>           ikChains;
    ResArray<Mouth>             mouths;
    ResArray<PoseParamDesc>     localPoseParams;
    ResString                   surfaceProp;
    ResStringView               keyValuesText;
    ResArray<IKLock>            localIKAutoplayLocks;
    f32                         mass;
    i32                         contents;
    ResArray<ModelGroup>        includeModels;
    i32                         unused_virtualModel;
    ResString                   animBlockName;
    ResArray<AnimBlock>         animBlocks;
    i32                         unused_animBlockModel;
    ResPtr<u8>                  animBlockModel;
    i32                         unused_pVertexBase;
    i32                         unused_pIndexBase;
    u8                          constDirectionalLightDot;
    u8                          rootLOD;
    u8                          numAllowedRootLODs;
    u8                          unused;
    i32                         unused4; // zero out if version < 47
    ResArray<FlexControllerUI>  flexControllerUIs;
    f32                         vertAnimFixedPointScale;
    mutable i32                 surfacePropLookup;
    ResPtr<Header2>             hdr2;
    i32                         unused2;
  };
} // namespace mdl

//-----------------------------------------------------------------------------
// .VVD File Format (derived from public/studio.h)
// Contains the global vertex list
//-----------------------------------------------------------------------------
namespace vvd {
  static constexpr ID32 ID      = "IDSV";
  static constexpr ID32 ID_THIN = "IDCV";
  static constexpr ID32 ID_NULL = "IDDV";
  static constexpr i32  VERSION = 4;

  // mstudioboneweight_t
  struct BoneWeights {
    f32 weight[MAX_NUM_BONES_PER_VERT];
    u8  bone[MAX_NUM_BONES_PER_VERT];
    u8  numBones;
  };
  static_assert(sizeof(BoneWeights) == 16);

  // mstudiovertex_t
  struct Vertex {
    BoneWeights boneWeights;
    vec3        position;
    vec3        normal;
    vec2        texCoord;
  };
  static_assert(sizeof(Vertex) == 48);

  // vertexFileFixup_t
  struct VVDFixup {
    i32 lod;
    i32 sourceVertexID;
    i32 numVertexes;
  };
  
  // vertexFileHeader_t
  struct Header {
    i32 id;
    i32 version;
    i32 checksum;
    i32 numLODs;
    i32 numLODVertexes[MAX_NUM_LODS];
    ResArray<VVDFixup>  fixups;
    ResPtr<Vertex>      vertexData;
    ResPtr<vec4>        tangentData;

    const Vertex& getVertex(size_t i) const {
      return *vertexData.get(this, i);
    }
  };
} // namespace vvd

//-----------------------------------------------------------------------------
// .VTX File Format (derived from public/optimize.h)
// Contains hardware optimized model data (vertex and index buffers)
//-----------------------------------------------------------------------------
namespace vtx {

#pragma pack(1)

  // OptimizedModel::Vertex_t
  struct Vertex {
    u8 boneWeightIndex[MAX_NUM_BONES_PER_VERT];
    u8 numBones;
    u16 origMeshVertID;
    u8 boneID[MAX_NUM_BONES_PER_VERT];
  };

  // OptimizedModel::BoneStateChangeHeader_t
  struct BoneStateChange {
    i32 hardwareID;
    i32 newBoneID;
  };

  // OptimizedModel::StripHeader_t
  struct Strip {
    enum Flags : u8 {
      TriList       = 1,
      QuadList      = 2,
      QuadListExtra = 4
    };
    i32 numIndices;
    i32 indexOffset;
    i32 numVerts;
    i32 vertOffset;
    i16 numBones;
    Flags flags;
    ResArray<BoneStateChange> boneStateChanges;
    i32 numTopologyIndices;
    i32 topologyOffset;
  };

  // OptimizedModel::StripGroupHeader_t
  struct StripGroup {
    enum Flags : u8 {
      HWSkinned       = 2,
      DeltaFlexed     = 4,
      SuppressHWMorph = 8
    };
    ResArray<Vertex>  vertices;
    ResArray<u16>     indices;
    ResArray<Strip>   strips;
    Flags             flags;
    ResArray<u16>     topologyIndices;
  };

  // OptimizedModel::MeshHeader_t
  struct Mesh {
    enum Flags : u8 {
      Teeth = 1,
      Eyes  = 2
    };
    ResArray<StripGroup> stripGroups;
    Flags flags;
  };

  // OptimizedModel::ModelLODHeader_t
  struct ModelLOD {
    ResArray<Mesh> meshes;
    f32 switchPoint;
  };

  // OptimizedModel::ModelHeader_t
  struct Model {
    ResArray<ModelLOD> lods;
  };

  // OptimizedModel::BodyPartHeader_t
  struct BodyPart {
    ResArray<Model> models;
  };

  // OptimizedModel::MaterialReplacementHeader_t
  struct MaterialReplacement {
    i16       materialID;
    ResString replacementMaterialName;
  };

  // OptimizedModel::MaterialReplacementListHeader_t
  struct MaterialReplacementList {
    ResArray<MaterialReplacement> replacements;
  };

  // OptimizedModel::FileHeader_t
  struct Header {
    static constexpr i32 VERSION = 7;
    i32 version;
    i32 vertCacheSize;
    u16 maxBonesPerStrip;
    u16 maxBonesPerFace;
    i32 maxBonesPerVert;
    i32 checkSum;
    i32 numLODs;
    ResPtr<MaterialReplacementList> materialReplacementList; // one per LOD
    ResArray<BodyPart> bodyParts;
  };

#pragma pack()

} // namespace vtx


//-----------------------------------------------------------------------------
using MDLHeader = mdl::Header;
using VVDHeader = vvd::Header;
using VTXHeader = vtx::Header;

using Buffer = std::span<u8>;

class ModelData {
public:
  ModelData(Buffer mdl) : mdlbuf(mdl) {
    this->mdl = reinterpret_cast<mdl::Header*>(mdlbuf.data());
  }
  ModelData(Buffer mdl, Buffer vvd) : mdlbuf(mdl), vvdbuf(vvd) {
    this->mdl = reinterpret_cast<mdl::Header*>(mdlbuf.data());
    this->vvd = reinterpret_cast<vvd::Header*>(vvdbuf.data());
  }
  ModelData(Buffer mdl, Buffer vvd, Buffer vtx) : mdlbuf(mdl), vvdbuf(vvd), vtxbuf(vtx) {
    this->mdl = reinterpret_cast<mdl::Header*>(mdlbuf.data());
    this->vvd = reinterpret_cast<vvd::Header*>(vvdbuf.data());
    this->vtx = reinterpret_cast<vtx::Header*>(vtxbuf.data());
  }

  const MDLHeader& getMDL() const { return *mdl; }
  const VVDHeader& getVertices() const { return *vvd; }
  const VTXHeader& getMeshData() const { return *vtx; }

protected:
  MDLHeader* mdl = nullptr;
  VVDHeader* vvd = nullptr;
  VTXHeader* vtx = nullptr;
  Buffer mdlbuf, vvdbuf, vtxbuf;
};


} // namespace libmdl
