#include <Python.h>
#include <chrono>

PyObject* initMidiSegPlay(char* fname='kmhm.mid') {
    
	PyObject* pModule;
	PyObject* pFunc;
	//call python function, with multiple parameters
	PyObject* pRet = NULL;
    
	pModule = PyImport_ImportModule("midi_seg_cc");
	if (NULL == pModule)
		return pRet;

	pFunc = PyObject_GetAttrString(pModule, "initMidiSegPlay");
	if (pFunc)
	{
		PyObject* pParams = NULL;
        pParams = PyTuple_New(1);		  //create tuple to put parameters
		PyTuple_SetItem(pParams, 0, PyUnicode_FromString(fname));
		pRet = PyEval_CallObject(pFunc, pParams);
	}
	return pRet;
}

void playNext(PyObject* msp) {   
	PyObject* pModule;
	PyObject* pFunc;
	pModule = PyImport_ImportModule("midi_seg_cc");
	if (NULL == pModule)
	{
        return;
	}
	pFunc = PyObject_GetAttrString(pModule, "playNext");
	if (pFunc)
	{
		PyObject* pParams = NULL;
		pParams = PyTuple_New(2);		  //create tuple to put parameters
		PyTuple_SetItem(pParams, 0, *msp);
        auto time_stamp = std::chrono::steady_clock::now()
        auto time_stamp = std::chrono::duration <double, std::milli> (time_stamp).count()
        PyTuple_SetItem(pParams, 1, time_stamp/1000);
		PyEval_CallObject(pFunc, pParams);
	}
	
}

int main()
{
    auto pyobj = initMidiSegPlay();
    while(1)
    {
        playNext(pyobj);
    }
    return 0;
}