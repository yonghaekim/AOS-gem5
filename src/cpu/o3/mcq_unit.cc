#include "cpu/o3/isa_specific.hh"
#include "cpu/o3/mcq_unit_impl.hh"

// Force the instantiation of LDSTQ for all the implementations we care about.
template class MCQUnit<O3CPUImpl>;

