#include "vhdl_producer.hh"
#include <algorithm>
#include <functional>
#include <iostream>
#include <string>
#include "vhdl_producer.hh"
#include "xtended.hh"

int Vertex::input_counter = 0;
int Vertex::output_counter = 0;

// Constants to make the creation of input/output vertices easier to follow
const bool INPUT = true;
const bool OUTPUT = false;

/*
    This function verifies if adding an edge in _edges will create a cycle in the DAG graph
*/
bool VhdlProducer::willCreateCycle(int source, int target) {
        std::vector<bool> visited(_vertices.size(), false);

        std::function<bool(int, int, std::vector<bool>)> isReachable;
        isReachable = [&](int s, int t, std::vector<bool> vis) -> bool {
            if (s == t) return true;

            vis[s] = true;

            for (Edge e : _edges[s]) {
                if (!vis[e.target] && isReachable(e.target, t, vis)) {
                    return true;
                }
            }

            return false;
        };

        return isReachable(source, target, visited);
    }

/**
 * Transforms a given tree-representation of a signal to an equivalent signal DAG representation.
 */
void VhdlProducer::visit(Tree signal)
{
    int vertex_id = _vertices.size();
    int     i;
    int64_t i64;
    double r;
    Tree    x, y, z, t;

    // Handle recursive signals
    if (isProj(signal, &i, x)) {
        // Projections are transformed in specific input and output vertices.
        // We also need to create two edges: one to the recursive output, and another
        // to the vertex originally using the projection.
        _visit_stack.push(VisitInfo::make_recursive(vertex_id));
        // This allows us to keep track of which recursive output node to link to in the subtree.
        _virtual_io_stack.push(vertex_id + 1);
        _virtual_io_queue.push(vertex_id);   // used to apply the graph to the retiming

        if(isRec(x, z, t)) {
            int nb_def_proj = 0;

            while (!isNil(t)) {
                if(i == nb_def_proj) {
                    if(_recursive_defintion.count(z->node()) != 0) {
                        bool added = false;
                        for(auto def_proj : _recursive_defintion[z->node()]) {
                            if(def_proj.first == hd(t)) {
                                added = true;
                                _recursive_defintion[z->node()][nb_def_proj].second.first = vertex_id;
                                _recursive_defintion[z->node()][nb_def_proj].second.second = i;
                                break;
                            }
                        }
                        if(!added) {
                            _recursive_defintion[z->node()].push_back(std::make_pair(hd(t), std::make_pair(vertex_id, i)));
                        }
                    } else {
                        _recursive_defintion[z->node()].push_back(std::make_pair(hd(t), std::make_pair(vertex_id, i)));
                    }
                } else {
                    if(_recursive_defintion.count(z->node()) == 0) {
                        _recursive_defintion[z->node()].push_back(std::make_pair(hd(t), std::make_pair(std::nullopt, std::nullopt)));
                    }
                }
                t = tl(t);
                nb_def_proj++;
            }
        }

        // Creating two vertices, an output and an input, that are linked.
        addVertex(Vertex(signal, OUTPUT));
        addVertex(Vertex(signal, INPUT));

        // Then visiting the projected value.
        self(x);

        _visit_stack.pop();
    }
    // Recursive symbols are bypassed in the final graph.
    else if (isRec(signal, x, y)) {
        mapself(y);
    }
    // General case
    else {
        auto existing_id = searchNode(signal);
        if(!existing_id.has_value()){
            // Initialize a new vertex
            if(isSigDelay(signal, x, y)){
                _visit_stack.push(VisitInfo::make_delay(vertex_id));
                // If the last visited node is a "Proj", we will bypass the delay to correct the dupliclated delay of 1 after a recursion
                for (Tree b : signal -> branches()) {
                    if (isProj(b, &i, x)){
                        bypass.push_back(signal->hashkey());
                    }
                }
            }else{
                _visit_stack.push(VisitInfo(vertex_id));
            }
            addVertex(Vertex(signal));
        } 
        // If the signal was already seen before and our subtree goes to a recursive output, 
        else if(isSigDelay(signal, x, y) && fVisited.count(signal) != 0 && fVisited[signal] == 1){
            for (Tree b : signal -> branches()) {
                if (isProj(b, &i, x) && fVisited.count(b) != 0 && fVisited[b] == 1){
                    _visit_stack.push(existing_id.value());
                }
            }
        }
        // Then visit its children.
        SignalVisitor::visit(signal);
        // Finally, we create edges from the children to the current vertex.
        int current_vertex_id = _visit_stack.top().vertex_index;
        _visit_stack.pop();

        if (!_visit_stack.empty()) {
            VisitInfo last_visited   = _visit_stack.top();


            if(last_visited.is_recursive) {
                // search the right vertex_id of the corresponding recursive output from _recursive_defintion
                int recursive_vertex_id = -1;
                for(auto def_rec : _recursive_defintion) {
                    for(auto def_proj : def_rec.second) {
                        if(def_proj.first == signal) {
                            recursive_vertex_id = def_proj.second.first.value();
                            break;
                        }
                    }
                }
                if(recursive_vertex_id == -1) {
                    std::cout << "\033[31m" << "ERROR : there is a problem with recursive output, recursive output not found" << "\033[0m" << std::endl;
                } else {
                    if(!willCreateCycle(recursive_vertex_id, current_vertex_id)) {
                        _edges[current_vertex_id].push_back(Edge(recursive_vertex_id, 0, _vertices[recursive_vertex_id].propagation_delay));
                    } else {
                        std::cout << "\033[31m" << "2 - You were about to add a cycle to the DAG Graph (recursion and the mem)" << "\033[0m" << std::endl;
                    }
                }
            } else {
                // Normal case with no recursion
                if(!willCreateCycle(last_visited.vertex_index, current_vertex_id)) {
                    _edges[current_vertex_id].push_back(Edge(last_visited.vertex_index, 0, _vertices[current_vertex_id].propagation_delay));
                } else {
                    std::cout << "\033[31m" << "2 - You were about to add a cycle to the DAG Graph (recursion and the mem)" << "\033[0m" << std::endl;
                }
            }
            
            if(fFinished.count(signal) != 0 && fFinished[signal]) {
                return;
            }

            // We register all delays of constant value
            if (last_visited.is_delay){
                if(isSigInt(signal, &i)){
                    int last_visited_id= last_visited.vertex_index;
                    delays.insert({_vertices[last_visited_id].node_hash,_vertices[current_vertex_id].node_hash});
                } 
            }

            if (last_visited.is_recursive) {
                // checks if the recursive signal has to be bypassed or not
                bool add_edge = false;
                for(auto def_rec : _recursive_defintion) {
                    for(auto def_proj : def_rec.second) {
                        if(def_proj.second.first == last_visited.vertex_index && def_proj.second.second == def_rec.second.size()-1 && def_proj.first == signal) {
                            add_edge = true;
                            break;
                        }
                    }
                }

                if(add_edge) {
                    _visit_stack.pop();
                    if(!willCreateCycle(_visit_stack.top().vertex_index, current_vertex_id)) {
                        _edges[current_vertex_id].push_back(Edge(_visit_stack.top().vertex_index, 0, _vertices[current_vertex_id].propagation_delay));
                    } else {
                        std::cout << "\033[31m" << "3 - You were about to add a cycle to the DAG Graph (recursion and the mem)" << "\033[0m" << std::endl;
                    }
                    _visit_stack.push(last_visited);
                }
            }
        } else {
            // We're at a root node, which means it is an output.
            int output_id = _vertices.size();
            addVertex(Vertex(signal, OUTPUT));
            if(!willCreateCycle(output_id, current_vertex_id)) {
                _edges[current_vertex_id].push_back(Edge(output_id, 0, _vertices[current_vertex_id].propagation_delay));
            } else {
                std::cout << "\033[31m" << "4 - You were about to add a cycle to the DAG Graph (recursion and the mem)" << "\033[0m" << std::endl;
            }       
        }
    }    
}


void VhdlProducer::generate(std::ostream& out)
{
    auto max_cycles_to_output = 0;
    for (size_t vertex = 0; vertex < _vertices.size(); ++vertex) {
            auto cycles_to_vertex = cyclesFromInput(vertex);
            if (cycles_to_vertex > max_cycles_to_output) {
                max_cycles_to_output = cycles_to_vertex;
            }
    }
    auto container = VhdlCodeContainer(_name, _inputs_count, _outputs_count, max_cycles_to_output, {});
    
    generic_mappings(container);
    instantiate_components(container);
    map_ports(container);
    

    // Output to file
    out << container;
}

/**
 * CODE GENERATION
 */
void VhdlProducer::instantiate_components(VhdlCodeContainer& container)
{
    // We generate a new component for each vertex
    int i = 0;
    for (auto vertex : _vertices) {
        container.register_component(vertex, cyclesFromInput(i));
        ++i;
    }
}


void VhdlProducer::map_ports(VhdlCodeContainer& container)
{
    // Iterates over all edges to map ports accordingly
    for (auto it = _vertices.begin(); it != _vertices.end(); ++it) {
        auto vertex = *it;
        auto edges = _edges[it - _vertices.begin()];
        for (auto edge : edges) {
            container.connect(vertex, _vertices[edge.target], edge.register_count);
        }
    }
    
    // Generate output mappings with the right conversions
    container.convertOut();
}

void VhdlProducer::generic_mappings(VhdlCodeContainer& container)
{
    // We add to delays the nodes that don't have a constant delay value but the result of a whole subtree
    for (auto element : max_delays){
        auto delay_hash = element.first;
        auto max_delay = element.second;
        if (delays.find(delay_hash)==delays.end()){
            delays.insert({delay_hash, max_delay});
        }
    }

    // We transfer the delays value to VhdlCodeContainer
    for (auto element : delays){
        auto delay_hash = element.first;
        auto delay_value = element.second;
        // We remove the additional delay of 1 in the case of a recursion
        auto it = std::find(bypass.begin(), bypass.end(), delay_hash) != bypass.end();
        if (it){
            if (delay_value > 0){
                delay_value = delay_value-1;
            } 
        }
        container.fill_delays(delay_hash, delay_value);
    }
}


/**
 * NORMALIZATION
 */
void VhdlProducer::normalize()
{
    // For each vertex `v`, we find the weight `w` of the longest incoming path.
    // The notion of weight is defined here as the sum of pipeline stages of vertices along the path.
    std::vector<std::optional<int>> max_incoming_weight(_vertices.size(), std::optional<int>());

    auto transposed_graph = transposedGraph();

    std::function<int(int)> incoming_weight;
    incoming_weight = [&](int vertex) {
        if (max_incoming_weight[vertex].has_value()) {
            return *max_incoming_weight[vertex];
        }

        max_incoming_weight[vertex] = std::make_optional(0);

        for (auto edge : transposed_graph[vertex]) {
            int w = incoming_weight(edge.target);
            if (w > *max_incoming_weight[vertex]) {
                *max_incoming_weight[vertex] = w;
            }
        }
        max_incoming_weight[vertex] = std::make_optional(_vertices[vertex].pipeline_stages + max_incoming_weight[vertex].value_or(0));
        return *max_incoming_weight[vertex];
    };

    for (size_t i = 0; i < _vertices.size(); ++i) {
        incoming_weight(i);
    }

    // Afterwards, for each vertex `u` such that there is an edge `u -> v`, we add `w - l(u)` registers
    // to `u -> v` to compensate for the lag.
    for (size_t u = 0; u < _vertices.size(); ++u) {
        for (Edge& edge : _edges[u]) {
            int max_pipeline_stages = *max_incoming_weight[edge.target] - _vertices[edge.target].pipeline_stages;
            edge.register_count += max_pipeline_stages - _vertices[u].pipeline_stages;
        }
    }
}

/**
 * RETIMING
 */


std::vector<int> topologicalOrdering(size_t vertices, const std::vector<std::vector<Edge>>& edges) {
    std::vector<int> stack;

    std::vector<bool> visited = std::vector<bool>(vertices, false);
    std::vector<bool> recursionStack = std::vector<bool>(vertices, false);
    bool hasCycle = false;

    std::function<void(int)> topologicalSort;
    topologicalSort = [&](int vertex) -> void {
      if (hasCycle) return;

      visited[vertex] = true;
      recursionStack[vertex] = true;

      for (auto adjacent : edges[vertex]) {
          if (recursionStack[adjacent.target]) {
                hasCycle = true;
                return;
            }
          if (!visited[adjacent.target]) {
              topologicalSort(adjacent.target);
          }
      }

    recursionStack[vertex] = false;
      stack.push_back(vertex);
    };

    for (size_t vertex = 0; vertex < vertices; ++vertex) {
        if (hasCycle) break;
        if (!visited[vertex]) {
            topologicalSort(vertex);
        }
    }

    if (hasCycle) {
        return {};
    }

    return stack;
}



void VhdlProducer::applyRegistersToGraph(Tree signals_retimed, Tree signals, Tree sig_last_visited) {
    // we visit in parallel the retimed and the original signal and we add the number of registers to the edge in the right place
    std::function<int(void)> getOutputVertexId;
    getOutputVertexId = [&](void) -> int {
        return _vertices.size() - 1;
    };

    int n;
    Tree x, y, z, yy, zz;

    if(isSigRegister(signals_retimed, &n, x) && !fVisitedForRetiming.count(signals)) {
        // if it is a register, we add the registers to the edge
        fVisitedForRetiming[signals] = 1;
        int nb_registers = n;
        auto vertex_id = searchNode(signals);
        if(vertex_id.has_value()) {
            // if the last visited signal is not null, that means that we will add registers to a normal signal that is not an output or a recursive signal
            if(sig_last_visited != nullptr) {
                auto vertex_last_visited_id = searchNode(sig_last_visited);
                if(vertex_last_visited_id.has_value()){
                    int k = 0;
                    for(Edge edge : _edges[vertex_id.value()]) {
                        if(edge.target == vertex_last_visited_id.value()) {
                            // we add the registers to the edge
                            _edges[vertex_id.value()][k].register_count += nb_registers;
                            break;
                        }
                        k++;
                    }
                } else {
                    std::cout << "\033[31m" << "ERROR: No last visited vertex found" << "\033[0m" << std::endl;
                }
            } 
            // sig_last_visited is null, that means we are at an output node (the root or a recursive output)
            else {
                if(_retiming_recursive_output) {
                    // The signal is a recursive output, we need to find the corresponding virtual io
                    // the virtual io is the first element in the queue of _virtual_io_queue
                    if(_virtual_io_queue.empty()) {
                        std::cout << "\033[31m" << "ERROR: No virtual io found" << "\033[0m" << std::endl;
                    } else {
                        // we get the vertex_id of the recursive output
                        auto output_id = _virtual_io_queue.front();
                        _virtual_io_queue.pop();
                        int k = 0;
                        for(Edge edge : _edges[vertex_id.value()]) {
                            if(edge.target == output_id) {
                                // we add the registers to the edge
                                _edges[vertex_id.value()][k].register_count += nb_registers;
                                break;
                            }
                            k++;
                        }
                        _retiming_recursive_output = false;
                    }
                } else {
                    // The signal is the root output
                    // we get the vertex_id of the output
                    auto output_id = getOutputVertexId();
                    if(output_id != -1){
                        int k = 0;
                        for(Edge edge : _edges[vertex_id.value()]) {
                            if(edge.target == output_id) {
                                // we add the registers to the edge
                                _edges[vertex_id.value()][k].register_count += nb_registers;
                                break;
                            }
                            k++;
                        }
                    } else {
                        std::cout << "\033[31m" << "ERROR: No output vertex found" << "\033[0m" << std::endl;
                    }
                }
            }
        } else {
            std::cout << "\033[31m" << "ERROR: No vertex found" << "\033[0m" << std::endl;
        }
        
        // after adding the registers to the edge, we visit the subtrees
        visitToApplyRetiming(x, signals, sig_last_visited);
    } else if(isRec(signals_retimed, y, z) && isRec(signals, yy, zz) && !fVisitedForRetiming.count(signals)) {
        // if it is a recursion, we get the definition of the signal and we visit it
        fVisitedForRetiming[signals] = 1;
        mapselfToApplyRetiming(z, zz);
    } else if(!fVisitedForRetiming.count(signals)){
        // if it is not a register or a recursion, we visit the signal
        fVisitedForRetiming[signals] = 1;
        visitToApplyRetiming(signals_retimed, signals, sig_last_visited);
    }
    return;
}

void VhdlProducer::mapselfToApplyRetiming(Tree signals_retimed, Tree signals)
{
    if (!isNil(signals_retimed) && !isNil(signals)) {
        _retiming_recursive_output = true;
        applyRegistersToGraph(hd(signals_retimed), hd(signals), nullptr);
        mapselfToApplyRetiming(tl(signals_retimed), tl(signals));
    }
}

void VhdlProducer::applyRetiming(Tree signals_retimed, Tree signals)
{
    while (!isNil(signals) && !isNil(signals_retimed)) {
        // we visit the retimed and the original signal in parallel
        applyRegistersToGraph(hd(signals_retimed), hd(signals), nullptr);
        signals = tl(signals);
        signals_retimed = tl(signals_retimed);
    }
}

static const char* binopname[] = {"+", "-", "*", "/", "%", "<<", ">>", ">", "<", ">=", "<=", "==", "!=", "&", "|", "^"};

/* return the label of a signal as a string
 strings*/
std::string VhdlProducer::sigLabel(const Tree sig) const
{
    int    i;
    double r;
    Tree   size, gen, wi, ws, tbl, ri, x, y, z, c, type, name, file, ff, largs, le, sel, var, label;

    xtended* p = (xtended*)getUserData(sig);

    std::stringstream fout;

    if (p) {
        fout << p->name();
    } else if (isSigInt(sig, &i)) {
        fout << i;
    } else if (isSigReal(sig, &r)) {
        fout << r;
    } else if (isSigWaveform(sig)) {
        fout << "waveform";
    }

    else if (isSigInput(sig, &i)) {
        fout << "INPUT_" << i;
    }
    else if ( isSigOutput(sig, &i, x) )        
         { fout << "OUTPUT_" << i; }

    else if (isSigDelay1(sig, x)) {
        fout << "mem";
    } else if (isSigDelay(sig, x, y)) {
        fout << "@";
    } else if (isSigPrefix(sig, x, y)) {
        fout << "prefix";
    } else if (isSigBinOp(sig, &i, x, y)) {
        fout << binopname[i];
    } else if (isSigFFun(sig, ff, largs)) {
        fout << "ffunction:" << *ff;
    } else if (isSigFConst(sig, type, name, file)) {
        fout << *name;
    } else if (isSigFVar(sig, type, name, file)) {
        fout << *name;
    }

    else if(isSigSin(sig)) {
        fout << "sin";
    }
    else if (isSigWRTbl(sig, size, gen, wi, ws)) {
        fout << "write:" << sig;
    } else if (isSigRDTbl(sig, tbl, ri)) {
        fout << "read";
    }

    else if (isSigSelect2(sig, sel, x, y)) {
        fout << "select2";
    }

    else if (isSigGen(sig, x)) {
        fout << "generator";
    }

    else if (isProj(sig, &i, x)) {
        fout << "Proj" << i;
    } else if (isRec(sig, var, le)) {
        fout << "REC " << *var;
    }

    else if (isSigIntCast(sig, x)) {
        fout << "int";
    } else if (isSigBitCast(sig, x)) {
        fout << "bit";
    } else if (isSigFloatCast(sig, x)) {
        fout << "float";
    }
#if 0
    else if ( isSigButton(sig, label) ) 			{ fout << "button \"" << *label << '"'; }
    else if ( isSigCheckbox(sig, label) ) 			{ fout << "checkbox \"" << *label << '"'; }
    else if ( isSigVSlider(sig, label,c,x,y,z) )	{ fout << "vslider \"" << *label << '"';  }
    else if ( isSigHSlider(sig, label,c,x,y,z) )	{ fout << "hslider \"" << *label << '"';  }
    else if ( isSigNumEntry(sig, label,c,x,y,z) )	{ fout << "nentry \"" << *label << '"';  }
    
    else if ( isSigVBargraph(sig, label,x,y,z) )	{ fout << "vbargraph \"" << *label << '"'; 	}
    else if ( isSigHBargraph(sig, label,x,y,z) )	{ fout << "hbargraph \"" << *label << '"'; 	}
#else
    else if (isSigButton(sig, label)) {
        fout << "button";
    } else if (isSigCheckbox(sig, label)) {
        fout << "checkbox";
    } else if (isSigVSlider(sig, label, c, x, y, z)) {
        fout << "vslider";
    } else if (isSigHSlider(sig, label, c, x, y, z)) {
        fout << "hslider";
    } else if (isSigNumEntry(sig, label, c, x, y, z)) {
        fout << "nentry";
    }

    else if (isSigVBargraph(sig, label, x, y, z)) {
        fout << "vbargraph";
    } else if (isSigHBargraph(sig, label, x, y, z)) {
        fout << "hbargraph";
    }
#endif
    else if (isSigAttach(sig, x, y)) {
        fout << "attach";
    }

    else if (isSigAssertBounds(sig, x, y, z)) {
        fout << "assertbounds";
    }

    else if (isSigLowest(sig, x)) {
        fout << "lowest";
    }

    else if (isSigHighest(sig, x)) {
        fout << "highest";
    }

    else {
        std::stringstream error;
        error << "ERROR : sigToGraph.cpp, unrecognized signal : " << *sig << std::endl;
        throw faustexception(error.str());
    }

    return fout.str();
}

void VhdlProducer::exportGraph(std::ostream& out) const
{
    out << "digraph {" << std::endl;
    for (size_t i = 0; i < _vertices.size(); ++i) {
      out << "\"" << std::hex << _vertices[i].node_hash << "_" << std::dec << i << "\" [label=<" << "<BR /><FONT POINT-SIZE=\"12\">id: " << i << ", " 
        << _vertices[i].signal->node() << " " ;
      
	// pipeline stage does not work
	// << ", pipeline stages: " << _vertices[i].pipeline_stages
        if (( _vertices[i].signal->node().type() != kIntNode) && (_vertices[i].signal->node().type() != kDoubleNode))
            out << VhdlProducer::sigLabel(_vertices[i].signal);
        out << "</FONT>>, weight=\"" << _vertices[i].pipeline_stages << "\"];" << std::endl;
        for (auto edge : _edges[i]) {
            out << "\"" << std::hex << _vertices[i].node_hash << "_"  << std::dec << i << "\" -> \"" << std::hex << _vertices[edge.target].node_hash << std::dec << "_" << edge.target << "\" [label=\"" << edge.register_count << "\",weight=\"" << edge.register_count << "\"];" << std::endl;
        }
    }

    out << "}" << std::endl;
}

void VhdlProducer::parseCustomComponents(std::istream& input)
{
    std::string id_str;
    std::string implementation_file;
    std::string pipeline_stages_str;

    while (!input.eof()) {
        std::getline(input, id_str, ' ');
        std::getline(input, implementation_file, ' ');
        std::getline(input, pipeline_stages_str, ';');

        _vertices[std::stoi(id_str)].pipeline_stages = std::stoi(pipeline_stages_str);
    }
}

int VhdlProducer::cyclesFromInput(int vertex) const
{
    std::vector<int> topological_order = topologicalOrdering(_vertices.size(), _edges);
    std::vector<std::vector<Edge>> transposed_graph = transposedGraph();

    // Then find the path with maximum weight to each vertex
    std::vector<int> incoming_weight = std::vector<int>(_vertices.size(), 0);
    std::function<int(int)> computeIncomingWeight;
    computeIncomingWeight = [&](int vertex_id) -> int {
      if (incoming_weight[vertex_id] != 0) {
          return incoming_weight[vertex_id];
      }


      std::vector<Edge> incoming_edges = transposed_graph[vertex_id];
      int max_incoming = 0;
      for (auto edge : incoming_edges) {
          int incoming = computeIncomingWeight(edge.target) + (_vertices[vertex_id].is_output() ? 0 : edge.register_count);
          if (incoming > max_incoming) {
              max_incoming = incoming;
          }
      }

      incoming_weight[vertex_id] = _vertices[vertex_id].pipeline_stages + max_incoming;
      return incoming_weight[vertex_id];
    };

    for (auto vertex : topological_order) {
        computeIncomingWeight(vertex);
    }

    return incoming_weight[vertex];
}

void VhdlProducer::visitToApplyRetiming(Tree sig_retimed, Tree sig, Tree last_visited)
{
    // this function is used to visit the retimed and the original signal in parallel to apply the retiming (add the registers_count to the edges)
    int     i, ii, n;
    int64_t i64, ii64;
    double  r, rr;
    Tree size, sizee, gen, genn, wi, wii, ws, wss, tbl, tbll, ri, rii, c, cc, sel, sell, x, xx, y, yy, z, zz, u, uu, 
        v, vv, var, varr, le, lee, label, labell, ff, fff, largs, largss, type, typee, name, namee,
        file, filee, sf, sff;

    if (getUserData(sig_retimed) && getUserData(sig)) {
        for (int j = 0; j < sig_retimed->arity(); j++) {
            applyRegistersToGraph(sig_retimed->branch(j), sig->branch(j), sig);
        }
        return;
    } else if (isSigInt(sig_retimed, &i) && isSigInt(sig, &ii)) {
        return;
    } else if (isSigInt64(sig_retimed, &i64) && isSigInt64(sig, &ii64)) {
        return;
    } else if (isSigReal(sig_retimed, &r) && isSigReal(sig, &rr)) {
        return;
    } else if (isSigWaveform(sig_retimed) && isSigWaveform(sig)) {
        return;
    } else if (isSigInput(sig_retimed, &i) && isSigInput(sig, &ii)) {
        return;
    } else if (isSigOutput(sig_retimed, &i, x) && isSigOutput(sig, &ii, xx)) {
        applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isSigDelay1(sig_retimed, x) && isSigDelay1(sig, xx)) {
        applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isSigDelay(sig_retimed, x, y) && isSigDelay(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        return;
    } else if (isSigPrefix(sig_retimed, x, y) && isSigPrefix(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        return;
    } else if (isSigBinOp(sig_retimed, &i, x, y) && isSigBinOp(sig, &ii, xx, yy)) {
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        return;
    }

    // Foreign functions
    else if (isSigFFun(sig_retimed, ff, largs) && isSigFFun(sig, fff, largss)) {
        applyRegistersToGraph(largs, largss, sig);
        return;
    } else if (isSigFConst(sig_retimed, type, name, file) && isSigFConst(sig, typee, namee, filee)) {
        return;
    } else if (isSigFVar(sig_retimed, type, name, file) && isSigFVar(sig, typee, namee, filee)) {
        return;
    }

    // Tables
    else if (isSigWRTbl(sig_retimed, size, gen, wi, ws) && isSigWRTbl(sig, sizee, genn, wii, wss)) {
        applyRegistersToGraph(size, sizee, sig);
        applyRegistersToGraph(gen, genn, sig);
        if (wi != gGlobal->nil && wii != gGlobal->nil) {
            // rwtable
            applyRegistersToGraph(wi, wii, sig);
            applyRegistersToGraph(ws, wss, sig);
        }
        return;
    } else if (isSigRDTbl(sig_retimed, tbl, ri) && isSigRDTbl(sig, tbll, rii)) {
        applyRegistersToGraph(tbl, tbll, sig);
        applyRegistersToGraph(ri, rii, sig);
        return;
    }

    // Doc
    else if (isSigDocConstantTbl(sig_retimed, x, y) && isSigDocConstantTbl(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        return;
    } else if (isSigDocWriteTbl(sig_retimed, x, y, u, v) && isSigDocWriteTbl(sig, xx, yy, uu, vv)) {
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        applyRegistersToGraph(u, uu, sig);
        applyRegistersToGraph(v, vv, sig);
        return;
    } else if (isSigDocAccessTbl(sig_retimed, x, y) && isSigDocAccessTbl(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        return;
    }

    // Select2 (and Select3 expressed with Select2)
    else if (isSigSelect2(sig_retimed, sel, x, y) && isSigSelect2(sig, sell, xx, yy)) {
        applyRegistersToGraph(sel, sell, sig);
        applyRegistersToGraph(x, xx, sig);
        applyRegistersToGraph(y, yy, sig);
        return;
    }

    // Table sigGen
    else if (isSigGen(sig_retimed, x) && isSigGen(sig, xx)) {
        if (fVisitGen) {
            applyRegistersToGraph(x, xx, sig);
            return;
        } else {
            return;
        }
    }

    // recursive signals
    else if (isProj(sig_retimed, &i, x) && isProj(sig, &ii, xx)) {
        applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isRec(sig_retimed, var, le) && isRec(sig, varr, lee)) {
        applyRegistersToGraph(sig_retimed, sig, nullptr);
        return;
    }

    // Int, Bit and Float Cast
    else if (isSigIntCast(sig_retimed, x) && isSigIntCast(sig, xx)) {
        applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isSigBitCast(sig_retimed, x) && isSigBitCast(sig, xx)) {
        applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isSigFloatCast(sig_retimed, x) && isSigFloatCast(sig, xx)) {
        applyRegistersToGraph(x, xx, sig);
        return;
    }

    // UI
    else if (isSigButton(sig_retimed, label) && isSigButton(sig, labell)) {
        return;
    } else if (isSigCheckbox(sig_retimed, label) && isSigCheckbox(sig, labell)) {
        return;
    } else if (isSigVSlider(sig_retimed, label, c, x, y, z) && isSigVSlider(sig, labell, cc, xx, yy, zz)) {
        applyRegistersToGraph(c, cc, sig), applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig), applyRegistersToGraph(z, zz, sig);
        return;
    } else if (isSigHSlider(sig_retimed, label, c, x, y, z) && isSigHSlider(sig, labell, cc, xx, yy, zz)) {
        applyRegistersToGraph(c, cc, sig), applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig), applyRegistersToGraph(z, zz, sig);
        return;
    } else if (isSigNumEntry(sig_retimed, label, c, x, y, z) && isSigNumEntry(sig, labell, cc, xx, yy, zz)) {
        applyRegistersToGraph(c, cc, sig), applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig), applyRegistersToGraph(z, zz, sig);
        return;
    } else if (isSigVBargraph(sig_retimed, label, x, y, z) && isSigVBargraph(sig, labell, xx, yy, zz)) {
        applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig), applyRegistersToGraph(z, zz, sig);
        return;
    } else if (isSigHBargraph(sig_retimed, label, x, y, z) && isSigHBargraph(sig, labell, xx, yy, zz)) {
        applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig), applyRegistersToGraph(z, zz, sig);
        return;
    }

    // Soundfile length, rate, buffer
    else if (isSigSoundfile(sig_retimed, label) && isSigSoundfile(sig, labell)) {
        return;
    } else if (isSigSoundfileLength(sig_retimed, sf, x) && isSigSoundfileLength(sig, sff, xx)) {
        applyRegistersToGraph(sf, sff, sig), applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isSigSoundfileRate(sig_retimed, sf, x) && isSigSoundfileRate(sig, sff, xx)) {
        applyRegistersToGraph(sf, sff, sig), applyRegistersToGraph(x, xx, sig);
        return;
    } else if (isSigSoundfileBuffer(sig_retimed, sf, x, y, z) && isSigSoundfileBuffer(sig, sff, xx, yy, zz)) {
        applyRegistersToGraph(sf, sff, sig), applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig), applyRegistersToGraph(z, zz, sig);
        return;
    }

    // Attach, Enable, Control
    else if (isSigAttach(sig_retimed, x, y) && isSigAttach(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig);
        return;
    } else if (isSigEnable(sig_retimed, x, y) && isSigEnable(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig);
        return;
    } else if (isSigControl(sig_retimed, x, y) && isSigControl(sig, xx, yy)) {
        applyRegistersToGraph(x, xx, sig), applyRegistersToGraph(y, yy, sig);
        return;
    }

    else if (isSigRegister(sig_retimed, &n, x)) {
        applyRegistersToGraph(sig_retimed, sig, last_visited);
        return;
    }

    else if (isNil(sig_retimed) || isNil(sig)) {
        // now nil can appear in table write instructions
        return;
    } else {
        std::cerr << __FILE__ << ":" << __LINE__ << " ASSERT : unrecognized signal : " << *sig_retimed << " and " << *sig << std::endl;
        faustassert(false);
    }
}



