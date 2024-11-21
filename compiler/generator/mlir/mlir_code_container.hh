#ifndef _MLIR_CODE_CONTAINER_H
#define _MLIR_CODE_CONTAINER_H

#include <llvm-19/llvm/Support/raw_ostream.h>
#include "code_container.hh"
#include "dsp_factory.hh"
#include "faust/Dialect.h"
#include "faust/Ops.h"
#include "faust/Types.h"
#include "mlir/IR/BuiltinAttributes.h"
#include "mlir/IR/ImplicitLocOpBuilder.h"
#include "mlir/IR/Location.h"
#include "mlir/IR/MLIRContext.h"
#include "mlir/IR/BuiltinTypes.h"
#include "signalVisitor.hh"
#include "signals.hh"
#include "llvm/Support/raw_os_ostream.h"

/**
 * @brief 
 * 
 */
struct MLIRSignalVisitor: public SignalVisitor 
{
    ::mlir::ModuleOp mod;
    ::faust::GraphOp graph;
    /**
     * @brief Get an MLIR Operation builder from the current context.
     * 
     * @return ImplicitLocOpBuilder 
     */
    ImplicitLocOpBuilder builder() {
        auto loc = mlir::UnknownLoc::get(mod->getContext());
        return mlir::ImplicitLocOpBuilder::atBlockEnd(
            loc,mod.getBody()
        );
    }
    /**
     * @brief Initializes the MLIR IR, building a top-level
     * builtin module as well as a 'faust.graph' region, passing
     * 'numInputs' signal types as arguments, returning 'numOutputs'
     * signal types as results.
     * 
     * @param numInputs 
     * @param numOutputs 
     */
    void initialize(int numInputs, int numOutputs) {
        mlir::MLIRContext ctx;
        ctx.loadDialect<faust::FaustDialect>();
        auto loc = mlir::UnknownLoc::get(&ctx);
        mod = mlir::ModuleOp::create(loc);
        auto b = builder();
        SmallVector<faust::SignalType> ins;
        SmallVector<faust::SignalType> outs;
        for (int n = 0; n < numInputs; ++n) {
             ins.emplace_back(faust::getSignalType());
        }
        for (int n = 0; n < numOutputs; ++n) {
             outs.emplace_back(faust::getSignalType());
        }
        graph = b.create<faust::GraphOp>(
            // add input signal types,
            ins, 
            b.getStringAttr("process"), 
            b.getArrayAttr({}),
            // add output signal graphs,
            outs                        
        );
    }
    /**
     * @brief Parses and builds a Faust signal into a MLIR SSA value.
     * 
     * @param sig Faust signal to be parsed.
     * @param b ImplicitLocOpBuilder object to build the SSA value.
     * @return mlir::Value 
     */
    mlir::Value parseOperation(Tree sig, ImplicitLocOpBuilder& b) 
    {
        int     i;
        int64_t i64;
        double  r;
        Tree    x, y, z, t;

        if (isSigInt(sig, &i)) {
            return b.create<faust::ConstantOp>(
                faust::getSignalType(),
                b.getI32IntegerAttr(i)
            );
        } else if (isSigInt64(sig, &i64)) {
            return b.create<faust::ConstantOp>(
                faust::getSignalType(),
                b.getI64IntegerAttr(i64)
            );
        } else if (isSigReal(sig, &r)) {
            return b.create<faust::ConstantOp>(
                faust::getSignalType(),
                // TODO: parse global graph precision
                b.getF32FloatAttr(r)
            );
        } else if (isSigDiv(sig, x, y)) {
            return b.create<faust::DivOp>(
                faust::getSignalType(),
                parseOperation(x, b),
                parseOperation(y, b)
            );
        } 
        assert(false && "Unimplemented operation type");
    }

    /**
     * @brief 
     * 
     * @param sig 
     */
    void visit(Tree sig) override {
        int i;
        Tree x;
        auto b = builder();
        // Set the cursor in the graph's region:
        b.setInsertionPointToStart(graph->getBlock());
        // Now parse the signal graph, and add operations:
        if (isSigOutput(sig, &i, x)) {
            b.create<faust::OutputOp>(
                parseOperation(x, b)
            );            
        } else {
            parseOperation(sig, b);
        }
    }
    /**
     * @brief Print IR in given ostream
     * 
     * @param path 
     */
    void print(std::ostream& out) const {
        auto os = llvm::raw_os_ostream(out);
        mod->print(os);        
    }
};

#endif



