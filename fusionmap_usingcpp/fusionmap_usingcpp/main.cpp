#include "drivable_area_detection.h"


//void foo(py::array_t<float> points_1d, py::list fusionmap) {
//	std::unique_ptr<DrivableAreaDetector> dd(new DrivableAreaDetector());
//	//DrivableAreaDetector dd;
//	dd->GetGridMap(points_1d, fusionmap);
//
//}


PYBIND11_MODULE(fusionmap_usingcpp, m) {

	py::class_<DrivableAreaDetector>(m, "DrivableAreaDetector")
		.def(py::init<>())
		.def("GetGridMap", &DrivableAreaDetector::GetGridMap)
		.def("Initialize", &DrivableAreaDetector::GridMapInitialize);

	m.doc() = "pybind11 example module";

	// Add bindings here
	//m.def("foo",&foo,"fusionmap",py::return_value_policy::copy);

}


