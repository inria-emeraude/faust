#ifndef _MLIR_CODE_CONTAINER_H
#define _MLIR_CODE_CONTAINER_H

#include "code_container.hh"
#include "dsp_factory.hh"
#include "faust/Dialect.h"


class MLIRCodeContainer: public virtual CodeContainer {
public:
    MLIRCodeContainer(const std::string& name, int numInputs, int numOutputs, std::ostream* out) 
    {
        initialize(numInputs, numOutputs);
        fKlassName = name;
        printf("Hello MLIR!\n");
    }
    
    virtual ~MLIRCodeContainer() {}

    void produceInternal() override {}

    CodeContainer* createScalarContainer(const std::string& name, int sub_container_type) override 
    {
        // TODO!
        return nullptr; 
    }
    static CodeContainer* createContainer(const std::string& name, int numInputs, int numOutputs,
                                          std::ostream* dst = new std::stringstream()) 
    {
        return new MLIRCodeContainer(name, numInputs, numOutputs, dst);
    }
};

#endif



