#include "fast_lio/common.h"

#include <fstream>

namespace cba {
int mysign(double x){ return (x > 0) - (x < 0); }

std::ostream &operator<<(std::ostream &os, const StampedPose &p) {
  os << p.time << " ";
  std::streamsize oldPrecision = os.precision();
  os << std::fixed << std::setprecision(7)
     << p.r[0] << " " << p.r[1] << " " << p.r[2] << " ";
  os << std::setprecision(9)
     << p.q.x() << " " << p.q.y() << " " << p.q.z() << " " << p.q.w();
  os.precision(oldPrecision);
  return os;
}

std::string StampedPose::toString() const {
  std::stringstream ss;
  const std::string delimiter(" ");
  ss << time << delimiter << std::setprecision(10) << r.x() << delimiter << r.y() << delimiter << r.z()
     << delimiter << q.x() << delimiter << q.y() << delimiter << q.z()
     << delimiter << q.w();
  return ss.str();
}

std::ostream &operator<<(std::ostream &os, const ScanMatch &p) {
  os << "(" << p.query << ',' << p.train << ')';
  return os;
}

std::ostream& operator<<(std::ostream& os, const KeyscanId& k) {
  os << k.t << " " << k.idx << " " << k.rangeStartIdx
     << " " << k.rangeFinishIdx;
  for (auto wid : k.windowIdx)
    os << " " << wid;
  return os;
}

std::ostream& operator<<(std::ostream& os, const ScanId& id) {
  os << id.t << " " << id.idx;
  return os;
}

Eigen::Vector3d IncrementMotion::ratio(const IncrementMotion &ref) const {
  Eigen::Vector3d rqv;
  rqv[0] = ref.pij.norm() * mysign(pij.dot(ref.pij)) / pij.norm();
  Eigen::AngleAxisd aar(ref.qij);
  Eigen::AngleAxisd aa(qij);
  rqv[1] = std::fabs(aar.angle() / aa.angle());
  rqv[2] = ref.vij.norm() * mysign(vij.dot(ref.vij)) / vij.norm();
  return rqv;
}

void IncrementMotion::multiply(const Eigen::Vector3d &coeffs) {
  pij *= coeffs[0];
  Eigen::AngleAxisd aa(qij);
  Eigen::Vector3d log = aa.axis() * aa.angle() * coeffs[1];
  Eigen::AngleAxisd saa(log.norm(), log.normalized());
  qij = saa;
  vij *= coeffs[2];
}

IncrementMotion StampedState::between(const StampedState &sj) const {
  IncrementMotion im;
  im.pij = q.conjugate() * (sj.r - r);
  im.qij = q.conjugate() * sj.q;
  im.vij = q.conjugate() * (sj.v - v);
  return im;
}

Eigen::Isometry3d StampedState::betweenPose(const StampedState& sj) const {
    Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond dq = q.conjugate() * sj.q;
    dT.linear() = dq.toRotationMatrix();
    Eigen::Vector3d dr = sj.r - r;
    dT.translation() = q.conjugate() * dr;
    return dT;
}

std::string StampedState::toString() const {
  const std::string delimiter = " ";
  std::stringstream os;
  os << time << delimiter << std::fixed << std::setprecision(6) << r[0] << delimiter << r[1] << delimiter << r[2];
  os << delimiter << std::fixed << std::setprecision(9) << q.x() << delimiter << q.y() << delimiter << q.z() << delimiter << q.w();
  os << delimiter << std::fixed << std::setprecision(6) << v[0] << delimiter << v[1] << delimiter << v[2];
  os << delimiter << std::fixed << std::setprecision(6) << bg[0] << delimiter << bg[1] << delimiter << bg[2];
  os << delimiter << std::fixed << std::setprecision(6) << ba[0] << delimiter << ba[1] << delimiter << ba[2];
  os << delimiter << std::fixed << std::setprecision(6) << gW[0] << delimiter << gW[1] << delimiter << gW[2];
  return os.str();
}

void StampedState::retract(const IncrementMotion &im) {
  r += q * im.pij;
  v += q * im.vij;
  q = q * im.qij;
}

std::ostream &operator<<(std::ostream &os, const StampedState &s) {
  os << s.toString();
  return os;
}

std::istream& operator>>(std::istream& is, StampedState& s) {
  std::vector<std::string> strs;
  std::string str;
  while(getline(is, str, ' '))
    strs.push_back(str);
  std::pair<uint32_t, uint32_t> ptime = parseTime(strs[0]);
  ros::Time rtime(ptime.first, ptime.second);
  std::vector<double> nums;
  nums.reserve(strs.size() - 1);
  for(size_t j=1; j<strs.size(); j++)
    nums.push_back(std::stod(strs[j]));
  s.time = rtime;
  int i = 0;
  s.r = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  s.q = Eigen::Quaterniond(nums[i+3], nums[i], nums[i+1], nums[i+2]);
  s.q.normalize();
  i += 4;
  s.v = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  s.bg = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  s.ba = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  s.gW = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  return is;
}

size_t loadStates(const std::string &stateFile,
                 StampedStateVector *states) {
  std::ifstream inFile(stateFile);
  if(!inFile.is_open()) {
    printf("Open state file %s failed.\n", stateFile.c_str());
    return 0;
  }
  states->reserve(100 * 10);
  std::string lineStr, str;
  while(getline(inFile, lineStr)) {
    if (lineStr[0] == '#')
      continue;
    std::stringstream ss(lineStr);
    StampedState s;
    ss >> s;
    states->push_back(s);
  }
  inFile.close();

  std::cout << "#Poses: " << states->size() << std::endl;
  std::cout << "First state: " << states->front() << std::endl;
  std::cout << "Last state: " << states->back() << std::endl;
  return states->size();
}

std::ostream& operator<<(std::ostream& os, const ImuData& d) {
  os << d.time << ", g: " << d.g.transpose()
     << ", a: " << d.a.transpose();
  return os;
}

const std::vector<uint32_t> power10 = makePower10();

std::pair<uint32_t, uint32_t> parseTime(const std::string &time) {
  size_t pos = time.find("."), pos1;
  size_t pad = 0, pow = 1;
  if (pos == std::string::npos) {
    pos = time.length() - 9;
    pos1 = pos;
  } else {
    pos1 = pos + 1;
    pad = 9 - (time.length() - pos1);
    pow = power10[pad];
  }
  std::string trunk = time.substr(0, pos);
  uint32_t sec, nsec;
  std::istringstream ss1(trunk);
  ss1 >> sec;

  std::string residuals = time.substr(pos1);
  std::istringstream ss2(residuals);
  ss2 >> nsec;
  nsec *= pow;
  return std::make_pair(sec, nsec);
}

void testParseTime() {
  auto p1 = parseTime("100000.100001");
  ros::Time t1(p1.first, p1.second);
  auto p2 = parseTime("100000100001000");
  ros::Time t2(p2.first, p2.second);
  std::cout << "0 == " << t1 - t2 << std::endl;
}

std::istream& operator>>(std::istream& is, ImuData& d) {
  std::vector<std::string> strs;
  std::string str;
  while(getline(is, str, ' '))
    strs.push_back(str);
  std::pair<uint32_t, uint32_t> ptime = parseTime(strs[0]);
  ros::Time rtime(ptime.first, ptime.second);
  std::vector<double> nums;
  nums.reserve(strs.size() - 1);
  for(size_t j=1; j<strs.size(); j++)
    nums.push_back(std::stod(strs[j]));
  d.time = rtime;
  int i = 0;
  d.g = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  d.a = Eigen::Vector3d(nums[i], nums[i+1], nums[i+2]);
  i += 3;
  return is;
}


} // namespace cba
