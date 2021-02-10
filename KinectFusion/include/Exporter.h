#ifndef EXPORTER_H
#define EXPORTER_H

#include "Util.h"
#include "Volume.h"

#include <string>

namespace cat {

namespace kf {

class Exporter {
public:
    static void exportMCMesh(const Volume& volume, const std::string fileName);
};

} // namespace kf
    
} // namespace cat

#endif