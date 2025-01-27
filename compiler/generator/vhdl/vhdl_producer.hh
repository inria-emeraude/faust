/************************************************************************
************************************************************************
FAUST compiler
Copyright (C) 2003-2018 GRAME, Centre National de Creation Musicale
---------------------------------------------------------------------
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or
(at your option) any later version.


This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.


You should have received a copy of the GNU Lesser General Public License
along with this program; if not, write to the Free Software
     	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
************************************************************************
************************************************************************/


#pragma once
#include "vhdl_code_container.hh"
#include "global.hh"
#include "sigtyperules.hh"
#include "signalVisitor.hh"


#include "occurrences.hh"
#include "recursivness.hh"
#include "normalform.hh"
#include "occurrences.hh"


#include <fstream>
#include <optional>
#include <iostream>
#include <string>
#include <queue>


typedef std::vector<int> Retiming;
// Target sample rate in kHz
const float FPGA_SAMPLE_RATE = 44.1;
// Target clock frequency in kHz
const float MASTER_CLOCK_FREQUENCY = 667000;

/**
* A wrapper around signals, with additional information such as propagation delay of pipeline stages
*/
struct Vertex {
   static int input_counter;
   static int output_counter;
   Tree signal;
   size_t node_hash;
   int nature;


   int propagation_delay = 1;
   int pipeline_stages = 0;


   bool recursive;


   Vertex(const Tree& signal)
   	: signal(signal), node_hash(signal->hashkey()), nature(getCertifiedSigType(signal)->nature()),
    	propagation_delay(1), pipeline_stages(0), recursive(false) {};


   // Creates an output/input node from another signal
   // This node can be a recursive output if it is linked to a Proj signal
   Vertex(const Tree& signal, bool is_input): Vertex(signal) {
   	int i;
   	Tree group;
   	if (!isProj(signal, &i, group)) {
       	i = is_input ? input_counter++ : output_counter++;
   	} else {
       	recursive = true;
   	}
	// we add 100 to the index to differentiate between input/output nodes that aren't recursive
   	this->signal = is_input ? sigInput(100 + i) : sigOutput(100 + i, signal);
   }


   bool is_output() const
   {
   	return signal->node() == gGlobal->SIGOUTPUT;
   }
   bool is_input() const {
   	return signal->node() == gGlobal->SIGINPUT;
   }
   bool is_recursive() const {
   	return recursive;
   }


   bool is_sin() const {
   	return isSigSin(signal);
   }
   int get_nature() const {
   	return nature;
   }
   Tree getTreeSignal() const {
   	return signal;
   }
};


template<>
struct std::hash<Vertex> {
   std::size_t operator()(Vertex const& v) const noexcept
   {
   	return v.node_hash;
   }
};


/**
* Structure holding information about connections between vertices, notably the number of registers between them.
* It also keeps intermediate results in memory, such as the highest critical path weight/delay
*/
struct Edge {
   int target;                            	//position of the target node in _vertices
   int register_count;


   int critical_path_weight;
   int critical_path_delay;


   Edge(int target_id, int register_count, int origin_delay): target(target_id), register_count(register_count),
   critical_path_weight(register_count), critical_path_delay(-origin_delay) {}


   bool operator==(const Edge& other) const {
   	return (target == other.target);
   }
};


/**
* Used to make the creation of a graph from the depth-first traversal of the signal tree clearer.
*/
struct VisitInfo {
   int vertex_index;
   bool is_recursive = false;
   bool is_delay = false;


   static VisitInfo make_recursive(int vertex_index) {
   	VisitInfo info(vertex_index);
   	info.is_recursive = true;
   	return info;
   }


   static VisitInfo make_delay(int vertex_index) {
   	VisitInfo info(vertex_index);
   	info.is_delay = true;
   	return info;
   }




   VisitInfo(int vertex_index): vertex_index(vertex_index) {}
};


//-------------------------VhdlProducer---------------------------------
// Transforms a signal into semantically equivalent VHDL code
//----------------------------------------------------------------------
class VhdlProducer : public SignalVisitor {
   // Graph
   std::vector<Vertex> _vertices;      	// Contains all the visited nodes
   std::vector<std::vector<Edge>> _edges;  // Contains the target of the node at the same position of the node in _vertices


   // Used to stock delays' information
   std::map <size_t, int> delays;      	// Maps the delay's hash to the delay value
   std::vector <size_t> bypass;        	// Contains the delays' nodes with a nill delay value
   std::map <size_t, int> max_delays;  	// Contains the delays' nodes with the max delay value taking into account the whole subtree arriving to the delay node


   // Used to create the graph from a signal tree
   std::stack<VisitInfo> _visit_stack;
   std::stack<int> _virtual_io_stack;
   std::map<int, std::vector<std::pair<Tree, std::pair<std::optional<int>, std::optional<int>>>>> _recursive_defintion;    // std::map<recursive_node, std::vector<Tree, pair<vertex_id, def_proj>>>

   // Used to apply retiming to the graph
   std::queue<int> _virtual_io_queue;
   bool _retiming_recursive_output = false;


   // General IP information
   std::string _name;
   int _inputs_count;
   int _outputs_count;
   Tree _signal;


   /** Visits the signal tree recursively to transform it into a weighted graph */
   virtual void visit(Tree signal) override;


  public:
   VhdlProducer(Tree signal, const std::string& name, int numInputs, int numOutputs)
   : _signal(signal), _name(name), _inputs_count(numInputs), _outputs_count(numOutputs), fOccMarkup(nullptr)
   {
        // Marking the tree in order to retreive the max delays 
        Tree L1 = _signal;
        recursivnessAnnotation(L1);
        typeAnnotation(L1, true);
        if (fOccMarkup!= nullptr) delete fOccMarkup;
        fOccMarkup = new OccMarkup();
        fOccMarkup->mark(L1); 

        // Convert the input signal to a weighted circuit graph
   	    visitRoot(_signal);
   }


   void addVertex(Vertex v)
   {
   	_vertices.push_back(v);
   	_edges.push_back({});
   }


   /*
   	This function verifies if adding an edge in _edges will create a cycle in the DAG graph
   */
   bool willCreateCycle(int source, int target);



   /** Applies a given retiming to the circuit, assuming said retiming is legal */
	void applyRetiming(Tree signals_retimed, Tree signals);


   /** Generates the VHDL code corresponding to the graph representation */
   void generate(std::ostream& out);


   /** Exports the graph as a DOT language file */
   void exportGraph(std::ostream& out) const;




   std::string sigLabel(const Tree sig) const;
 
   // Starts the Faust tree visit
   void initializeFromSignal() {
   	// Marking the tree in order to retreive the max delays
   	Tree L1 = _signal;
   	recursivnessAnnotation(L1);
   	typeAnnotation(L1, true);
   	if (fOccMarkup!= nullptr) delete fOccMarkup;
   	fOccMarkup = new OccMarkup();
   	fOccMarkup->mark(L1);


   	// Convert the input signal to a weighted circuit graph
   	visitRoot(_signal);


   	// Parse the components file to get pipeline stages information (optional)
   	if (!gGlobal->gVHDLComponentsFile.empty()) {
       	std::ifstream components_file(gGlobal->gVHDLComponentsFile);
       	if (!components_file) {
           	std::cerr << "ASSERT : failed to read file : " << gGlobal->gVHDLComponentsFile << std::endl;
           	faustassert(false);
       	}
       	parseCustomComponents(components_file);
       	components_file.close();


       	// We only need to normalize the graph if we're using user-defined components
       	normalize();
   	}
   }


  protected:

    // apply register_count to the graph
   void applyRegistersToGraph(Tree signals_retimed, Tree signals, Tree sig_last_visited);

	// for recursion
   void mapselfToApplyRetiming(Tree signals_retimed, Tree signals);

   // visit the tree signal to apply the retiming to the graph
   void visitToApplyRetiming(Tree sig_retimed, Tree sig, Tree last_visited);


   // Contains occurence of all signals
   OccMarkup*  fOccMarkup;
 


   /**
	* CODE GENERATION
	*/
   // For each Vertex of _vertices we generate the right VHDL code depending on the node's type
   void instantiate_components(VhdlCodeContainer& container);
   // Fills a variable of VhdlCodeContainer with, for each node of the Tree, the  sources nodes including registers with the right register count
   void map_ports(VhdlCodeContainer& container);
   // Fills a variable of VhdlCodeContainer with the delays' hash and their delay value
   void generic_mappings(VhdlCodeContainer& container);


   /**
	* NORMALIZATION
	*/
   /**
	* Normalizes the circuit, adding registers to compensate the eventual difference in
	* lag induces by pipelined operators
	*/
   void normalize();



   /**
	* HELPER FUNCTIONS
	*/
   /** Computes the maximum number of cycles necessary to access a given vertex.
	* It is equivalent to the longest path along the graph, weighted by registers and pipeline stages.
	* This is useful to propagate signals like ap_start in the actual circuit along a series of registers.
	*/
   int cyclesFromInput(int vertex) const;


// get the vertex index of a signal and returns it if it exists in the graph from the hash
   std::optional<int> searchNode(const size_t hash) const {
   	for (size_t v = 0; v < _vertices.size(); ++v) {
       	if (_vertices[v].node_hash == hash) {
           	return std::optional<int>(v);
       	}
   	}


   	return std::nullopt;
   }



	// get the vertex index of a signal and returns it if it exists in the graph
   std::optional<int> searchNode(const Tree sig) const {
   	for (int v = 0; v < _vertices.size(); ++v) {
       	if ((_vertices[v].signal) == sig) {
           	return std::optional<int>(v);
       	}
   	}


   	return std::nullopt;
   }


   std::vector<int> incomingEdges(int vertex_id, const std::vector<std::vector<Edge>>& edges) const {
   	std::vector<int> incoming;
   	for (int v = 0; v < edges.size(); ++v) {
       	for (auto edge : edges[v]) {
           	if (edge.target == vertex_id) {
               	incoming.push_back(v);
           	}
       	}
   	}
   	return incoming;
   }


// Returns the transposed graph of the current graph
   std::vector<std::vector<Edge>> transposedGraph() const {
   	std::vector<std::vector<Edge>> transposed(_edges.size(), std::vector<Edge>());
   	for (size_t v = 0; v < _edges.size(); ++v) {
       	for (auto edge : _edges[v]) {
           	transposed[edge.target].push_back(Edge(v, edge.register_count, edge.critical_path_delay));
       	}
   	}
   	return transposed;
   }


   /** Parses a user-defined config file for operators
	* Such files are structured as follows:
	* <id> <implementation file> <pipeline stages>
	*  12	flopoco_fpadd.vhdl     	4
	*
	* To find the id of a component/vertex, you can first run a pass using the --vhdl-trace option
	* and find the id on the resulting vhdl_graph.dot file.
	*/
   void parseCustomComponents(std::istream& input);


 
   /** Overrides the TreeTraversal::self method to handle recursion */
 
   virtual void self(Tree t) override
   {  
   	int 	i;
   	Tree	x, y, l, cur, min, max, step;
 	 
   	// Sliders are considered as constant corresponding to their default value "cur"
   	if(isSigHSlider(t, l, cur, min, max, step)){
       	t = cur;
   	}
 	 
   	// Delays with value bigger than 0 are registered
   	if(!_visit_stack.empty()){
       	VisitInfo last_visited   = _visit_stack.top();
       	int vertex_id = _visit_stack.top().vertex_index;
       	auto delay_hash = _vertices[vertex_id].node_hash;
       	if(last_visited.is_delay){
           	if(delays.find(delay_hash) == delays.end()){
               	Occurrences* o = fOccMarkup->retrieve(t);
               	int delay_value = o->getMaxDelay();
               	if (delay_value > 0){
                   	max_delays.insert({delay_hash,delay_value});
               	}
           	}  
       	}
   	}

   	if (fTrace) traceEnter(t);
   	fIndent++;
 	 
   	if (!fVisited.count(t)) {
		// If the node is not already visited
       	fVisited[t] = 1;
       	visit(t);
 	 
   	}else if(isProj(t, &i, x)){
       	auto existing_id = searchNode(t->hashkey());
       	// If the signal was already seen before and our subtree goes to a recursive output,
       	// we add the corresponding recursive input to this node.
		_virtual_io_stack.push(existing_id.value() + 1);
       	if (existing_id.has_value() && !_virtual_io_stack.empty()) {
           	int vertex_id = _visit_stack.top().vertex_index;
           	int virtual_input_id = _virtual_io_stack.top();
           	_edges[virtual_input_id].push_back(Edge(vertex_id, 0, 0));
           	_virtual_io_stack.pop();
       	}
 	 
   	}else if(getVisitCount(t) == 1 && isSigDelay(t, x, y)){
		// if the node is a delay and we started visiting it but we haven't finished yet and it reappears in the tree, 
		// that means that it is used more than once so we have to finish visiting it before continuing
       	visit(t);
   	}else{
       	//if the node is already visited, we only establish an edge if not already done
       	size_t hash = t->hashkey();
       	auto existing_id = searchNode(hash);

       	if(existing_id.has_value() && !_visit_stack.empty()){
           	int vertex_id = _visit_stack.top().vertex_index;
           	int nb_edges_established = 0;
           	if(VisitInfo(_visit_stack.top()).is_delay) {
               	for(int i = 0; i < _vertices.size(); i++){
                   	for(auto edge : _edges[i]){
                       	if(edge.target == vertex_id){
                           	nb_edges_established++;
                       	}
                   	}
               	}
           	}
            // if the edge is not already established, we establish it
           	if(nb_edges_established < 2) {
               	_edges[existing_id.value()].push_back(Edge(vertex_id, 0, 0));
               	VisitInfo last_visited   = _visit_stack.top();
               	// If the last visited node is a delay and the current one is a constant, then we register the node
               	if (last_visited.is_delay){
                   	if(isSigInt(t, &i)){
                       	delays.insert({_vertices[vertex_id].node_hash,hash});
                   	}
               	}
           	}
       	} else if(isRec(t, x, y)) {
		   	// recursive definition already visited
			// we have to add an edge from the recursive input to the last vertex in the stack
			auto output_recursive_id = _visit_stack.top();
			_visit_stack.pop();
		   	int vertex_id = _visit_stack.top().vertex_index;
		   	int virtual_input_id = _virtual_io_stack.top();
		   	_edges[virtual_input_id].push_back(Edge(vertex_id, 0, 0));
			_virtual_io_stack.pop();
			_visit_stack.push(output_recursive_id);
	   	}
   	}


   	fFinished[t] = true;
     	 
   	// Keep visit counter
   	fVisited[t]++;
   	fIndent--;
   	if (fTrace) traceExit(t);


   }


   // Fonction to display the node type corresponding to a signal
   void displayNode(Tree sig){
   int 	i;
   int64_t i64;
   double  r;
   Tree	size, gen, wi, ws, tbl, ri, c, sel, x, y, z, u, v, var, le, label, ff, largs, type, name, file, sf;
 
   	if (isSigInt(sig, &i)) {
       	std::cout << "Int" << std::endl;
   	} else if (isSigInt64(sig, &i64)) {
       	std::cout << "Int64" << std::endl;
   	} else if (isSigReal(sig, &r)) {
       	std::cout << "Real" << std::endl;
   	} else if (isSigWaveform(sig)) {
       	std::cout << "waveform" << std::endl;
   	} else if (isSigInput(sig, &i)) {
       	std::cout << "Input" << std::endl;
   	} else if (isSigOutput(sig, &i, x)) {
       	std::cout << "output" << std::endl;
   	} else if (isSigDelay1(sig, x)) {
       	std::cout << "Delay1" << std::endl;
   	} else if (isSigDelay(sig, x, y)) {
       	std::cout << "Delay" << std::endl;
   	} else if(isSigSin(sig)) {
       	std::cout << "Sine" << std::endl;
   	} else if (isSigPrefix(sig, x, y)) {
       	std::cout << "Prefix" << std::endl;
   	} else if (isSigBinOp(sig, &i, x, y)) {
       	std::cout << "BinOp" << std::endl;
   	} else if (isProj(sig, &i, x)) {
       	std::cout << "Proj" << std::endl;
   	} else if (isRec(sig, var, le)) {
       	std::cout << "Rec" << std::endl;
   	} else if (isSigFloatCast(sig, x)) {
       	std::cout << "float Cast" << std::endl;
   	}else{
       	std::cout << "node not registered" << std::endl;
   	}
   }
 
};
