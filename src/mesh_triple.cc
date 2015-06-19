#include "mesh_triple.h"

MeshTriple::MeshTriple(Triple* triple) {
	this -> triple = triple;
}

bool MeshTriple::operator==(const MeshTriple &t1) {
	return (*(this->triple) == *(t1.triple));
}

bool MeshTriple::operator!=(const MeshTriple &t1) {
	return !(*this == t1);
}
