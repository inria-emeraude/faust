#ifndef _MLIR_CODE_CONTAINER_H
#define _MLIR_CODE_CONTAINER_H

#include <cstdlib>
#include <memory>
#include <vector>

#include "binop.hh"
#include "faust/mlir/Dialect.h"
#include "faust/mlir/Ops.h"
#include "faust/mlir/Types.h"

#include "mlir/IR/BuiltinAttributes.h"
#include "mlir/IR/ImplicitLocOpBuilder.h"
#include "mlir/IR/Location.h"
#include "mlir/IR/MLIRContext.h"
#include "mlir/IR/Value.h"

#include <llvm/Support/raw_os_ostream.h>
#include <llvm/ADT/ArrayRef.h>
#include <llvm/Support/raw_ostream.h>

#include "node.hh"
#include "signals.hh"
#include "tree.hh"
#include "xtended.hh"

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
    void initialize(int numInputs, int numOutputs) 
    {
        fContext.loadDialect<faust::FaustDialect>();
        auto loc = mlir::UnknownLoc::get(&fContext);
        fMod = mlir::ModuleOp::create(loc);

        fBuilder = std::make_unique<ImplicitLocOpBuilder>(
            ImplicitLocOpBuilder::atBlockEnd(loc, fMod.getBody())
        );
        // fBuilder->setInsertionPointToEnd(fMod.getBody());
        SmallVector<::mlir::NamedAttribute> ins;
        SmallVector<::mlir::Type> outs;
        faust::RealType rType = faust::RealType::get(&fContext);

        fGraph = fBuilder->create<faust::GraphOp>(
            // add input signal types,
            outs,
            fBuilder->getStringAttr("process"), 
            fBuilder->getDictionaryAttr({}),
            fBuilder->getI64IntegerAttr(numInputs),
            fBuilder->getI64IntegerAttr(numOutputs)                           
        );
        auto& block = fGraph.getBody().emplaceBlock();
        for (auto& _in: ins) {
             block.addArgument(rType, fBuilder->getLoc());
        }
        fBuilder->setInsertionPointToStart(&block);
    }

    template<typename T>
    mlir::Value createBinOp(Tree x, Tree y) {
        ImplicitLocOpBuilder& b = builder();
        return b.create<T>(
            visit(x), 
            visit(y)
        ).getResult();
    }

    template<typename T>
    mlir::Value createUnaryOp(Tree x) {
        ImplicitLocOpBuilder& b = builder();
        return b.create<T>(
            visit(x)
        ).getResult();
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

        std::cerr << "Visiting signal: ";
        sig->print(std::cerr);
        std::cerr << std::endl;

        // Integer (i32) constant
        if (isSigInt(sig, &i)) {
            return b.create<faust::IntOp>(
                b.getI32IntegerAttr(i)
            ).getResult();
        // Integer (i64) constant
        } else if (isSigInt64(sig, &i64)) {
            return b.create<faust::IntOp>(
                b.getI64IntegerAttr(i64)
            ).getResult();
        // // Real constant
        } else if (isSigReal(sig, &r)) {
            return b.create<faust::RealOp>(
                // TODO: parse global graph precision
                b.getF64FloatAttr(double(r))
            ).getResult();
        } else if (isSigDelay(sig, x, y)) {
            return b.create<faust::DelayOp>(
                visit(x),
                visit(y)
            ).getResult();
        // } else if (isSigDelay1(sig, x)) {
        //     return b.create<faust::DelayOp>(
        //         visit(x),
        //         b.getI32IntegerAttr(1)
        //     ).getResult();
        } else if (isProj(sig, &i, x)) {
            return b.create<faust::ProjOp>(
                visit(x),
                b.getI32IntegerAttr(i)
            ).getResult();
        } else if (isTree(sig, gGlobal->SYMREC, x)) {
            return b.create<faust::RecOp>(
                visit(x)
            );
        // ----------------------------------------------------------------
        // Unary / Math
        // ----------------------------------------------------------------
        } else if (isTree(sig, gGlobal->gAcosPrim->symbol(), x)) {
            return createUnaryOp<faust::AcosOp>(x);
        } else if (isTree(sig, gGlobal->gAsinPrim->symbol(), x)) {
            return createUnaryOp<faust::AsinOp>(x);
        } else if (isTree(sig, gGlobal->gAtanPrim->symbol(), x)) {
            return createUnaryOp<faust::AtanOp>(x);
        } else if (isTree(sig, gGlobal->gAtan2Prim->symbol(), x)) {
            return createUnaryOp<faust::Atan2Op>(x);
        } else if (isTree(sig, gGlobal->gCosPrim->symbol(), x)) {
            return createUnaryOp<faust::CosOp>(x);
        } else if (isTree(sig, gGlobal->gSinPrim->symbol(), x)) {
            return createUnaryOp<faust::SinOp>(x);
        } else if (isTree(sig, gGlobal->gTanPrim, x)) {
            return createUnaryOp<faust::TanOp>(x);
        } else if (isTree(sig, gGlobal->gExpPrim, x)) {
            return createUnaryOp<faust::ExpOp>(x);
        } else if (isTree(sig, gGlobal->gLogPrim->symbol(), x)) {
            return createUnaryOp<faust::LogOp>(x);
        } else if (isTree(sig, gGlobal->gLog10Prim->symbol(), x)) {
            return createUnaryOp<faust::Log10Op>(x);
        } else if (isTree(sig, gGlobal->gSqrtPrim->symbol(), x)) {
            return createUnaryOp<faust::SqrtOp>(x);
        } else if (isTree(sig, gGlobal->gAbsPrim->symbol(), x)) {
            return createUnaryOp<faust::AbsOp>(x);
        } else if (isTree(sig, gGlobal->gFloorPrim->symbol(), x)) {
            return createUnaryOp<faust::FloorOp>(x);
        } else if (isTree(sig, gGlobal->gCeilPrim->symbol(), x)) {
            return createUnaryOp<faust::CeilOp>(x);
        } else if (isTree(sig, gGlobal->gRintPrim->symbol(), x)) {
            return createUnaryOp<faust::RintOp>(x);
        } else if (isTree(sig, gGlobal->gRoundPrim->symbol(), x)) {
            return createUnaryOp<faust::RoundOp>(x);
        // ----------------------------------------------------------------
        // BinOps
        // ----------------------------------------------------------------
        } else if (isTree(sig, "pow", x, y)) {
            return createBinOp<faust::PowOp>(x, y);
        } else if (isTree(sig, "min", x, y)) {
            return createBinOp<faust::MinOp>(x, y);
        } else if (isTree(sig, "max", x, y)) {
            return createBinOp<faust::MaxOp>(x, y);
        } else if (isTree(sig, "fmod", x, y)) {
            return createBinOp<faust::ModOp>(x, y);
        } else if (isSigBinOp(sig, &i, x, y)) {
            #define FBINOP(_E, _T) case _E: return createBinOp<_T>(x, y)
            switch (i) {
                FBINOP(SOperator::kAdd, faust::AddOp);
                FBINOP(SOperator::kSub, faust::SubOp);
                FBINOP(SOperator::kDiv, faust::DivOp);
                FBINOP(SOperator::kMul, faust::MulOp);
                FBINOP(SOperator::kAND, faust::BitAndOp);
                FBINOP(SOperator::kOR, faust::BitOrOp);
                FBINOP(SOperator::kXOR, faust::BitXorOp);
                FBINOP(SOperator::kRem, faust::RemainderOp);
                FBINOP(SOperator::kEQ, faust::EqOp);
                FBINOP(SOperator::kNE, faust::NeqOp);
                FBINOP(SOperator::kLsh, faust::ShiftLeftOp);
                FBINOP(SOperator::kLRsh, faust::ShiftRightOp);
                FBINOP(SOperator::kGT, faust::SupOp);
                FBINOP(SOperator::kGE, faust::SupEqOp);
                FBINOP(SOperator::kLT, faust::InfOp);
                FBINOP(SOperator::kLE, faust::InfEqOp);
                default: 
                    std::cerr << "Unknown BinaryOp: " << i << std::endl;
                    exit(EXIT_FAILURE);
            }
        // Input signal:
        } else if (isSigInput(sig, &i)) {
            return b.create<::faust::InputOp>(
                b.getUI32IntegerAttr(i)
            ).getResult();
        } else if (isSigFloatCast(sig)) {
        }
        std::cerr << "Unimplemented signal type: ";
        sig->print(std::cerr);
        exit(EXIT_FAILURE);
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



