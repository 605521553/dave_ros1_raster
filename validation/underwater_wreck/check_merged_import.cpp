#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/Mesh.hh>
#include <iostream>
int main(int argc, char** argv) {
 for (int i=1; i<argc; ++i) {
  auto m=gazebo::common::MeshManager::Instance()->Load(argv[i]);
  if (!m) return 1;
  std::cout<<argv[i]<<" submeshes="<<m->GetSubMeshCount()
   <<" vertices="<<m->GetVertexCount()<<" indices="<<m->GetIndexCount()
   <<" min="<<m->Min()<<" max="<<m->Max()<<std::endl;
 }
}
