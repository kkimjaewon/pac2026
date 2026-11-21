// Copyright 2026 Jaewon Kim
#include "pac_hal/scene_guard.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <vector>
#include <filesystem>
#include <fcl/fcl.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

namespace pac_hal
{
namespace
{
using Transform = fcl::Transform3d;
using Shape = std::shared_ptr<fcl::CollisionGeometryd>;
Transform transform(const urdf::Pose & p)
{
  Transform t = Transform::Identity();
  t.linear() = Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y,
    p.rotation.z).toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  return t;
}
Transform transform(const KDL::Frame & f)
{
  Transform t = Transform::Identity();
  for (int i = 0; i < 3; ++i) {
    t.translation()[i] = f.p[i];
    for (int j = 0; j < 3; ++j) {t.linear()(i, j) = f.M(i, j);}
  }
  return t;
}
Shape mesh(const urdf::Mesh & m)
{
  std::string path;
  const std::string prefix = "package://";
  if (m.filename.rfind(prefix, 0) == 0) {
    const auto slash = m.filename.find('/', prefix.size());
    if (slash == std::string::npos) {throw std::runtime_error("invalid package mesh URI");}
    path = ament_index_cpp::get_package_share_directory(
      m.filename.substr(prefix.size(), slash - prefix.size())) + m.filename.substr(slash);
  } else if (m.filename.rfind("file:///", 0) == 0) {
    // Official UR xacro expands collision meshes to absolute local file URIs.
    path = m.filename.substr(7);
  } else {throw std::runtime_error("local package/file mesh URI required");}
  Assimp::Importer reader;
  const auto * scene = reader.ReadFile(path,
    aiProcess_Triangulate | aiProcess_PreTransformVertices | aiProcess_JoinIdenticalVertices);
  if (!scene) {throw std::runtime_error(reader.GetErrorString());}
  std::vector<fcl::Vector3d> vertices;
  std::vector<fcl::Triangle> triangles;
  for (unsigned i = 0; i < scene->mNumMeshes; ++i) {
    const auto * part = scene->mMeshes[i];
    const auto offset = vertices.size();
    for (unsigned j = 0; j < part->mNumVertices; ++j) {
      const auto & v = part->mVertices[j];
      vertices.emplace_back(v.x * m.scale.x, v.y * m.scale.y, v.z * m.scale.z);
    }
    for (unsigned j = 0; j < part->mNumFaces; ++j) {
      const auto & face = part->mFaces[j];
      if (face.mNumIndices != 3) {throw std::runtime_error("triangular collision mesh required");}
      triangles.emplace_back(offset + face.mIndices[0], offset + face.mIndices[1],
        offset + face.mIndices[2]);
    }
  }
  if (triangles.empty()) {throw std::runtime_error("empty collision mesh");}
  auto result = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
  result->beginModel(); result->addSubModel(vertices, triangles); result->endModel();
  return result;
}
Shape shape(const urdf::GeometrySharedPtr & g)
{
  if (!g) {throw std::runtime_error("missing collision geometry");}
  if (g->type == urdf::Geometry::MESH) {return mesh(*std::static_pointer_cast<urdf::Mesh>(g));}
  if (g->type == urdf::Geometry::BOX) {
    const auto b = std::static_pointer_cast<urdf::Box>(g);
    return std::make_shared<fcl::Boxd>(b->dim.x, b->dim.y, b->dim.z);
  }
  if (g->type == urdf::Geometry::CYLINDER) {
    const auto b = std::static_pointer_cast<urdf::Cylinder>(g);
    return std::make_shared<fcl::Cylinderd>(b->radius, b->length);
  }
  if (g->type == urdf::Geometry::SPHERE) {
    return std::make_shared<fcl::Sphered>(std::static_pointer_cast<urdf::Sphere>(g)->radius);
  }
  throw std::runtime_error("unsupported collision geometry");
}
std::pair<std::string, std::string> pair(std::string a, std::string b)
{
  return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}
bool collision(fcl::CollisionObjectd & a, fcl::CollisionObjectd & b)
{
  // AABB로 먼 쌍을 제외한 뒤 공식 메쉬의 상세 충돌 검사를 수행한다.
  if (!a.getAABB().overlap(b.getAABB())) {return false;}
  fcl::CollisionResultd result;
  fcl::collide(&a, &b, fcl::CollisionRequestd(), result);
  return result.isCollision();
}
}  // namespace

struct SceneGuard::Impl
{
  struct Body {std::string link; Transform origin; std::unique_ptr<fcl::CollisionObjectd> object;};
  struct Obstacle {std::string name; std::unique_ptr<fcl::CollisionObjectd> object;};
  Config config;
  bool enabled{false}, ready{false};
  double joint_step{.002};
  unsigned max_steps{128};
  KDL::Tree tree;
  KDL::JntArray joints;
  std::unique_ptr<KDL::TreeFkSolverPos_recursive> fk;
  Transform installation;
  std::vector<Body> bodies;
  std::vector<Obstacle> obstacles;
  std::set<std::pair<std::string, std::string>> ignored;
  std::set<std::pair<std::string, std::string>> environment_ignored;
  explicit Impl(const Config & c) : config(c) {}
};

SceneGuard::SceneGuard(const Config & c, const std::string & file) : impl_(std::make_unique<Impl>(c))
{
  if (file.empty()) {return;}
  auto & s = *impl_;
  const auto yaml = YAML::LoadFile(file);
  if (yaml["version"].as<int>() != 1 || yaml["frame"].as<std::string>() != c.reference) {
    throw std::runtime_error("collision scene version/frame mismatch");
  }
  const auto margin = yaml["obstacle_margin_m"].as<double>();
  s.joint_step = yaml["joint_step_rad"].as<double>();
  s.max_steps = yaml["max_path_steps"].as<unsigned>();
  if (!std::isfinite(margin) || margin < 0 || margin > .05 ||
    !std::isfinite(s.joint_step) || s.joint_step <= 0 || s.joint_step > .002 ||
    s.max_steps == 0 || s.max_steps > 4096) {throw std::runtime_error("invalid collision sampling");}
  const auto & p = c.reference_to_base;
  s.installation = Transform::Identity();
  s.installation.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
    p.orientation.y, p.orientation.z).toRotationMatrix();
  s.installation.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!yaml["boxes"].IsSequence() || yaml["boxes"].size() == 0) {
    throw std::runtime_error("collision scene needs obstacle boxes");
  }
  std::set<std::string> names;
  for (const auto & box : yaml["boxes"]) {
    const auto name = box["name"].as<std::string>();
    if (name.empty() || !names.insert(name).second) {throw std::runtime_error("duplicate obstacle name");}
    const auto xyz = box["center"].as<std::vector<double>>();
    const auto sizes = box["size"].as<std::vector<double>>();
    const auto q = box["quaternion_xyzw"].as<std::vector<double>>();
    if (xyz.size() != 3 || sizes.size() != 3 || q.size() != 4) {throw std::runtime_error("invalid box pose/size");}
    Transform t = Transform::Identity();
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(xyz[i]) || !std::isfinite(sizes[i]) || sizes[i] <= 0) {
        throw std::runtime_error("nonfinite/nonpositive box geometry");
      }
      t.translation()[i] = xyz[i];
    }
    for (size_t i = 0; i < s.config.joint_count; ++i) {if (!std::isfinite(q[i])) {throw std::runtime_error("invalid box quaternion");}}
    Eigen::Quaterniond rotation(q[3], q[0], q[1], q[2]);
    if (std::abs(rotation.norm() - 1.) > 1e-6) {throw std::runtime_error("box quaternion not normalized");}
    t.linear() = rotation.toRotationMatrix();
    auto geometry = std::make_shared<fcl::Boxd>(sizes[0] + 2 * margin,
      sizes[1] + 2 * margin, sizes[2] + 2 * margin);
    s.obstacles.push_back({name, std::make_unique<fcl::CollisionObjectd>(geometry, t)});
  }
  // Model-specific SRDF exclusions are supplied by the scene profile; tools are not automatically excluded.
  for (const auto & entry : yaml["disabled_self_pairs"]) {
    const auto names = entry.as<std::vector<std::string>>();
    if (names.size() != 2 || names[0] == names[1]) {throw std::runtime_error("invalid self pair");}
    s.ignored.insert(pair(names[0], names[1]));
  }
  for (const auto & entry : yaml["disabled_environment_pairs"]) {
    const auto names = entry.as<std::vector<std::string>>();
    if (names.size() != 2 || !names[0].size() || !names[1].size()) {
      throw std::runtime_error("invalid environment pair");
    }
    s.environment_ignored.insert({names[0], names[1]});
  }
  s.enabled = true;
}
SceneGuard::~SceneGuard() = default;
bool SceneGuard::enabled() const {return impl_->enabled;}
bool SceneGuard::ready() const {return !enabled() || impl_->ready;}

Result SceneGuard::initialize(const std::string & xml)
{
  auto & s = *impl_;
  if (!s.enabled || s.ready) {return {};}
  try {
    s.bodies.clear();  // Retry must not retain a partially loaded collision model.
    urdf::Model model;
    if (!model.initString(xml) || !kdl_parser::treeFromUrdfModel(model, s.tree)) {
      throw std::runtime_error("invalid collision URDF");
    }
    for (const auto & [a, b] : s.ignored) {
      if (!model.getLink(a) || !model.getLink(b)) {throw std::runtime_error("unknown disabled self link");}
    }
    for (const auto & [link, obstacle] : s.environment_ignored) {
      if (!model.getLink(link) || std::none_of(s.obstacles.begin(), s.obstacles.end(),
          [&obstacle](const auto & o) {return o.name == obstacle;})) {
        throw std::runtime_error("unknown environment exception");
      }
    }
    if (!model.getLink(s.config.base) || s.tree.getNrOfJoints() != s.config.joint_count) {
      throw std::runtime_error("configured fixed-tool joint count required");
    }
    s.joints.resize(s.config.joint_count);
    s.fk = std::make_unique<KDL::TreeFkSolverPos_recursive>(s.tree);
    for (const auto & [name, link] : model.links_) {
      // Only the configured robot subtree is a robot body. Simulator scenery
      // (e.g. UR xacro's ground_plane sibling) belongs to the world scene.
      auto ancestor = link;
      while (ancestor && ancestor->name != s.config.base) {ancestor = ancestor->getParent();}
      if (!ancestor) {continue;}
      for (const auto & c : link->collision_array) {
        s.bodies.push_back({name, transform(c->origin),
          std::make_unique<fcl::CollisionObjectd>(shape(c->geometry))});
      }
    }
    if (s.bodies.empty()) {throw std::runtime_error("robot collision model empty");}
    s.ready = true;
    return {};
  } catch (const std::exception & e) {return {Error::E_TF, e.what()};}
}

Result SceneGuard::check(const Joints & q)
{
  auto & s = *impl_;
  if (!s.enabled) {return {};}
  if (!s.ready) {return {Error::E_TF, "collision model not initialized"};}
  for (size_t i = 0; i < s.config.joint_count; ++i) {if (!std::isfinite(q[i])) {return {Error::E_FORMAT, "nonfinite collision joints"};}}
  for (const auto & [name, segment] : s.tree.getSegments()) {
    const auto & j = segment.segment.getJoint();
    if (j.getType() == KDL::Joint::Fixed) {continue;}
    auto found = std::find_if(s.config.axes.begin(), s.config.axes.begin() + s.config.joint_count,
      [&j](const auto & axis) {return axis.name == j.getName();});
    if (found == s.config.axes.begin() + s.config.joint_count) {return {Error::E_TF, "unknown collision joint"};}
    s.joints(segment.q_nr) = q[std::distance(s.config.axes.begin(), found)];
  }
  KDL::Frame base;
  if (s.fk->JntToCart(s.joints, base, s.config.base) < 0) {return {Error::E_TF, "collision base FK failed"};}
  const Transform installation = s.installation * transform(base).inverse();
  for (auto & body : s.bodies) {
    KDL::Frame frame;
    if (s.fk->JntToCart(s.joints, frame, body.link) < 0) {return {Error::E_TF, "collision FK failed"};}
    body.object->setTransform(installation * transform(frame) * body.origin);
    body.object->computeAABB();
    for (auto & obstacle : s.obstacles) {
      if (s.environment_ignored.count({body.link, obstacle.name})) {continue;}
      if (collision(*body.object, *obstacle.object)) {
        return {Error::E_OUT_OF_RANGE, "environment collision: " + body.link + " / " + obstacle.name};
      }
    }
  }
  for (size_t i = 0; i < s.bodies.size(); ++i) {
    for (size_t j = i + 1; j < s.bodies.size(); ++j) {
      auto & a = s.bodies[i]; auto & b = s.bodies[j];
      if (a.link == b.link || s.ignored.count(pair(a.link, b.link))) {continue;}
      if (collision(*a.object, *b.object)) {
        return {Error::E_OUT_OF_RANGE, "self collision: " + a.link + " / " + b.link};
      }
    }
  }
  return {};
}

Result SceneGuard::check_path(const Joints & from, const Joints & to, int64_t deadline)
{
  if (!enabled()) {return {};}
  double delta = 0.;
  for (size_t i = 0; i < impl_->config.joint_count; ++i) {
    if (!std::isfinite(from[i]) || !std::isfinite(to[i])) {return {Error::E_FORMAT, "nonfinite collision path"};}
    delta = std::max(delta, std::abs(to[i] - from[i]));
  }
  const auto steps = std::max(1., std::ceil(delta / impl_->joint_step));
  if (steps > impl_->max_steps) {return {Error::E_OUT_OF_RANGE, "collision path too large; split approach"};}
  for (unsigned k = 0; k <= static_cast<unsigned>(steps); ++k) {
    if (deadline && monotonic_ns() >= deadline) {return {Error::E_EXPIRED, "expired during collision check"};}
    Joints q{};
    for (size_t i = 0; i < impl_->config.joint_count; ++i) {q[i] = from[i] + (to[i] - from[i]) * k / steps;}
    const auto result = check(q);
    if (!result) {return result;}
  }
  return {};
}
}  // namespace pac_hal
