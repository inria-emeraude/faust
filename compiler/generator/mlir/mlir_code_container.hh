#ifndef _MLIR_CODE_CONTAINER_H
#define _MLIR_CODE_CONTAINER_H

#include <llvm-19/llvm/ADT/ArrayRef.h>
#include <llvm-19/llvm/Support/raw_ostream.h>
#include <memory>
#include <vector>
#include "code_container.hh"
#include "dsp_factory.hh"
#include "faust/Dialect.h"
#include "faust/Ops.h"
#include "faust/Types.h"
#include "mlir/IR/BuiltinAttributeInterfaces.h"
#include "mlir/IR/BuiltinAttributes.h"
#include "mlir/IR/ImplicitLocOpBuilder.h"
#include "mlir/IR/Location.h"
#include "mlir/IR/MLIRContext.h"
#include "mlir/IR/BuiltinTypes.h"
#include "mlir/IR/TypeRange.h"
#include "mlir/IR/Value.h"
#include "signalVisitor.hh"
#include "signals.hh"
#include "llvm/Support/raw_os_ostream.h"
#include "sigtyperules.hh"
#include "mlir/Dialect/Arith/IR/Arith.h"

/**
 * @brief Builds a Faust MLIR IR from a graph of signals.
 * 
 */
struct MLIRBuilder
{
    std::set<Tree> fVisited;
    ::mlir::ModuleOp fMod;
    ::mlir::MLIRContext fContext;
    ::faust::GraphOp fGraph;
    std::unique_ptr<ImplicitLocOpBuilder> fBuilder;

    /**
     * @brief Get an MLIR Operation builder from the current context.
     * 
     * @return Reference to ImplicitLocOpBuilder 
     */
    ImplicitLocOpBuilder& builder() {
        return *fBuilder;
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

        fContext.loadDialect<faust::FaustDialect>();
        auto loc = mlir::UnknownLoc::get(&fContext);
        fMod = mlir::ModuleOp::create(loc);

        fBuilder = std::make_unique<ImplicitLocOpBuilder>(
            ImplicitLocOpBuilder::atBlockEnd(loc, fMod.getBody())
        );
        // fBuilder->setInsertionPointToEnd(fMod.getBody());
        SmallVector<::mlir::NamedAttribute> ins;
        SmallVector<::mlir::Type> outs;
        auto fType = faust::SignalType::get(&fContext, fBuilder->getF32Type());
        for (int n = 0; n < numInputs; ++n) {
             auto in = fBuilder->getNamedAttr("in0", TypeAttr::get(fType));
             ins.push_back(in);
        }
        for (int n = 0; n < numOutputs; ++n) {
             // TODO: check float precision:
            outs.push_back(fType);            
        }
        fGraph = fBuilder->create<faust::GraphOp>(
            // add input signal types,
            outs,
            fBuilder->getStringAttr("process"), 
            fBuilder->getDictionaryAttr({}),
            // add output signal graphs,
            fBuilder->getDictionaryAttr(ins)                                 
        );
        auto& block = fGraph.getBodyRegion().emplaceBlock();
        for (auto& in: ins) {
            block.addArgument(fType, fBuilder->getLoc());
        }
        fBuilder->setInsertionPointToStart(&block);
    }

    template<typename T>
    mlir::Value createBinOp(Tree x, Tree y) {
        ImplicitLocOpBuilder& b = builder();
        return b.create<T>(
            faust::SignalType::get(b.getContext(), b.getF32Type()),
            visit(x), 
            visit(y)
        );
    }

    mlir::Value visit(Tree sig) {
        if (fVisited.count(sig)) {
            return {};
        } else {
            fVisited.insert(sig);
        }

        int     i;
        int64_t i64;
        double  r;
        Tree    x, y, z, t;

        ImplicitLocOpBuilder& b = builder();
        faust::SignalType fType = faust::SignalType::get(
            b.getContext(), b.getF32Type()
        );       
        
        if (isSigInt(sig, &i)) {
            return b.create<faust::ConstantOp>(
                fType,
                b.getI32IntegerAttr(i)
            );
        } else if (isSigInt64(sig, &i64)) {
            return b.create<faust::ConstantOp>(
                fType,
                b.getI64IntegerAttr(i64)
            );
        } else if (isSigReal(sig, &r)) {
            return b.create<faust::ConstantOp>(
                fType,
                // TODO: parse global graph precision
                b.getF32FloatAttr(float(r))
            );
        } else if (isSigBinOp(sig, &i, x, y)) {
            switch (i) {
                case SOperator::kAdd: {
                    return createBinOp<faust::AddOp>(x, y);
                }
                case SOperator::kSub: {
                    return createBinOp<faust::SubOp>(x, y);
                }                
                case SOperator::kDiv: {
                    return createBinOp<faust::DivOp>(x, y);
                }
                case SOperator::kMul: {
                    return createBinOp<faust::MulOp>(x, y);
                }
                default:
                assert(false);
            }
        } else if (isSigInput(sig, &i)) {
            return fGraph.getBodyRegion()
                .getBlocks()
                .front()
                .getArgument(i);
        } else if (isSigFloatCast(sig)) {
        }
        assert(false);
    }

    void build(Tree sig) {
        std::vector<mlir::Value> outs;
        if (isList(sig)) {
            do {
                outs.push_back(visit(hd(sig)));
                sig = tl(sig);        
            } while (isList(sig));
        } else {
            outs.push_back(visit(sig));
        }
        builder().create<faust::OutputOp>(outs);
    }

    /**
     * @brief Print MLIR IR in given 'out' ostream.
     * 
     * @param path 
     */
    void print(std::ostream& out) const {
        auto os = llvm::raw_os_ostream(out);
        fMod->print(os);        
    }
};

#endif



