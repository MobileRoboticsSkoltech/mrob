/* $COPYRIGHT SKOLTECH
 * $LICENSE_LGPL
 *
 * mrobPy.cpp
 *
 *  Created on: Jan 17, 2019
 *      Author: Gonzalo Ferrer
 *              g.ferrer@skoltech.ru
 *              Mobile Robotics Lab, Skoltech
 */


#include <pybind11/pybind11.h>
namespace py = pybind11;



void init_SE3(py::module &m);
void init_FGraph(py::module &m);
void init_PCRegistration(py::module &m);



PYBIND11_MODULE(mrob, m) {
    m.doc() = "pybind11 MROB library, now including \n-SE3\n-PCRegistration";
    // Later, in binding code:
    init_SE3(m);
    init_FGraph(m);
    init_PCRegistration(m);
}




