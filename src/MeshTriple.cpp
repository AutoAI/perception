bool MeshTriple::operator==(const MeshTriple &t1) {
	return (*(this->triple) == *(t1.triple));
}

bool MeshTriple::operator!=(const MeshTriple &t1) {
	return !(t1 == *this);
}
