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

#include "vhdl_code_container.hh"
#include "vhdl_producer.hh"
#include "binop.hh"
#include "sigtyperules.hh"
#include "xtended.hh"
#include <iomanip>
#include <sstream>


//
//*functions to print the final vhdl code*/
//

std::ostream& operator<<(std::ostream& out, const VhdlCodeBlock& block) {
    block._buffer.output_buffer(out);
    return out;
}

std::ostream& operator<<(std::ostream& out, const VhdlCodeContainer& container) {

    
    out << std::endl << "-- ======= ENTITIES =========" << std::endl;
    out << container._entities << std::endl;

    out << std::endl << "-- ======= FAUST IP =========" << std::endl;
   
    out << "library ieee;" << std::endl
        << "use ieee.std_logic_1164.all;" << std::endl
        << "use ieee.numeric_std.all;" << std::endl
        << "use ieee.std_logic_arith.all;" << std::endl
        << "use ieee.std_logic_signed.all;" << std::endl
        << "use work.fixed_float_types.all;" << std::endl;
    // Include the right package for real numbers encoding
    if (gGlobal->gVHDLFloatEncoding) {
        out << "use work.float_pkg.all;" << std::endl;
    } else {
        out << "use work.fixed_pkg.all;" << std::endl;
    }
        
    out << "entity " << container._ip_name << " is" << std::endl
        << "port (" << std::endl
        << '\t' << VhdlPort("ws", PortMode::Input, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl
        << '\t' << VhdlPort("ap_clk", PortMode::Input, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl
        << '\t' << VhdlPort("ap_rst_n", PortMode::Input, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl
        << '\t' << VhdlPort("ap_start", PortMode::Input, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl
        << '\t' << VhdlPort("bypass_dsp", PortMode::Input, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl
        << '\t' << VhdlPort("bypass_faust", PortMode::Input, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl
        << '\t' << VhdlPort("ap_done", PortMode::Output, VhdlType(VhdlInnerType::StdLogic)) << ";" << std::endl;

    for (size_t input = 0; input < container._num_inputs; ++input) {
        out << '\t' << VhdlPort("audio_in_" + std::to_string(input), PortMode::Input, VhdlType(VhdlInnerType::StdLogicVector, 23, 0)) << (input == container._num_inputs - 1 && container._num_outputs == 0 ? "" : ";") << std::endl;
    }
    for (size_t output = 0; output < container._num_outputs; ++output) {
        out << '\t' << VhdlPort("audio_out_" + std::to_string(output), PortMode::Output, VhdlType(VhdlInnerType::StdLogicVector, 23, 0)) << ";" << std::endl;
        out << '\t' << VhdlPort("audio_out_ap_vld_" + std::to_string(output), PortMode::Output, VhdlType(VhdlInnerType::StdLogic)) << (output == container._num_outputs - 1 ? "" : ";") << std::endl;
    }

    out << ");" << std::endl << "end " << container._ip_name << ";" << std::endl;

    out << "architecture DSP of "<< container._ip_name << " is" << std::endl;
    out << std::endl << "-- ======= SIGNALS ==========" << std::endl;
    out << container._signals << std::endl;

    //  Declare signals register_n
    for (size_t register_id = 0; register_id < container._register_series.size(); ++register_id) {
        auto register_series = container._register_series[register_id];
        out << "signal registers_" << register_id << " : " << register_series.type << ";" << std::endl;  
    }

    // Declare signals converted_register_n to connect components with different port types
    for (auto mapping : container._mappings) {
        if (mapping.second.empty()) {
            continue;
        }
        auto target_hash = mapping.first;
        // auto target_id = container._signal_identifier.at(target_hash);
        for (size_t i = 0; i < mapping.second.size(); ++i) {
            auto registers_id = mapping.second[i].second;
            if (registers_id) {
                auto register_series = container._register_series[registers_id];
                if (container.port_type.find(target_hash) != container.port_type.end()){
                    auto ports = container.port_type.at(target_hash);
                    if (i < ports.size()){
                        if(ports[i].type != register_series.type.type){
                            if(ports[i].type== VhdlInnerType::SFixed){
                                out << "signal converted_registers_" << registers_id << " : sfixed(8 downto -23) := to_sfixed(registers_" << registers_id << ", 8, -23);" << std::endl;
                            }else if(ports[i].type== VhdlInnerType::Integer){
                                out << "signal converted_registers_" << registers_id << " : integer := to_integer(registers_" << registers_id << ");" << std::endl;
                            } else if(ports[i].type== VhdlInnerType::RealFloat){
                                out << "signal converted_registers_" << registers_id << " : float(8 downto -23) := to_float(registers_" << registers_id << ", 8, 23);" << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
    out << std::endl << "-- ======= COMPONENTS =======" << std::endl;
    out << container._components << std::endl;

    out << std::endl << "begin" << std::endl;

    out << std::endl << "-- ======= DATA FLOW EQUATIONS IN =======" << std::endl;
    out << container._conversionIn << std::endl;
    
    out << std::endl << "-- ======= PORT MAPPINGS ====" << std::endl;


    for (auto mapping : container._mappings) {
        // Components that do not depend on anything are constants
        auto target_hash = mapping.first;
        
        if (mapping.second.empty() && (container._delays.find(target_hash)==container._delays.end())) {
            continue;
        }

        if(container._signal_identifier.count(target_hash) != 0) {
            auto target_id = container._signal_identifier.at(target_hash);
            out << "pm_" << target_id << " : " << entityTypeFromName(target_id) << std::endl;
            
            // For delays, we map n to the constant value
            if (!(container._delays.find(target_hash)==container._delays.end())){
                if (container._delays.at(target_hash) != 0){
                    out << '\t' << "generic map (n => " << container._delays.at(target_hash) << ")" << std::endl;
                }  
            }

            out << '\t' << "port map (" << std::endl;
            out << "\t\t" << "clock => ap_clk," << std::endl;
            out << "\t\t" << "reset => ap_rst_n," << std::endl;

            // Check if the component is a one sample delay and connect its port "write_enable"
            auto write_enable_sig = container._one_sample_delay_mappings.find(target_hash);
            if (write_enable_sig != container._one_sample_delay_mappings.end()) {
                if(write_enable_sig->second != -1){
                    out << "\t\t" << "write_enable => registers_" << write_enable_sig->second << "," << std::endl;
                } else {
                    out << "\t\t" << "write_enable => ap_start," << std::endl;
                }
            }

            // Check if the componenent is a delay and connect its port "clock_enable"
            auto clock_enable_sig = container._delays_mappings.find(target_hash);
            if (clock_enable_sig != container._delays_mappings.end()){
                if(container._delays_mappings.at(target_hash) == -1) {
                    out << "\t\t" << "clock_enable => ap_start," << std::endl;
                } else {
                    out << "\t\t" << "clock_enable => registers_" << container._delays_mappings.at(target_hash) << "," << std::endl;
                }
            }

            for (size_t i = 0; i < mapping.second.size(); ++i) {
                auto source_hash = mapping.second[i].first;
                auto registers_id = mapping.second[i].second;

                
                if (registers_id) {
                    auto register_series = container._register_series[registers_id];
                    if (container.port_type.find(target_hash) != container.port_type.end()){
                        auto ports = container.port_type.at(target_hash);
                        if (i < ports.size()){
                            if(ports[i].type == register_series.type.type){
                                out << "\t\t" << "data_in_" << i << " => " << "registers_" << registers_id << "," << std::endl; 
                            }else{
                                out << "\t\t" << "data_in_" << i << " => " << "converted_registers_" << registers_id << "," << std::endl;
                            }
                        }
                    }  
                } else {
                    out << "\t\t" << "data_in_" << i << " => " << container._signal_identifier.at(source_hash) << "," << std::endl;
                }
                
            }
            out << "\t\t" << "data_out => " << target_id << std::endl;
            out << "\t" << ");" << std::endl << std::endl;
        }
    }
    for (size_t register_id = 0; register_id < container._register_series.size(); ++register_id) {
        auto register_series = container._register_series[register_id];

        out << "pm_"<< register_series.name << " : " << entityTypeFromName(register_series.name) << std::endl;
        out << '\t' << "generic map (n => " << register_series.registers_count << ")" << std::endl;
        out << '\t' << "port map (" << std::endl;
        out << "\t\t" << "clock => ap_clk," << std::endl;
        out << "\t\t" << "reset => ap_rst_n," << std::endl;

        auto source = register_series.source.has_value() ? container._signal_identifier.at(*register_series.source) : "ap_start";
        
        out << "\t\t" << "data_in => " << source << "," << std::endl;
        out << "\t\t" << "data_out => registers_" << register_id << std::endl;
        out << '\t' << ");" << std::endl << std::endl;
    }

    // "registers_0" is reserved for the series of registers responsible for
    // delaying the "ap_start" signal received from the I2S
    if (container._cycles_per_sample > 0) {
        out << "ap_done <= registers_0;" << std::endl;
        for (size_t output = 0; output < container._num_outputs; ++output) {
            out << "audio_out_ap_vld_" << output << " <= registers_0;" << std::endl;
        }
    } else {
        out << "ap_done <= ap_start;" << std::endl;
        for (size_t output = 0; output < container._num_outputs; ++output) {
            out << "audio_out_ap_vld_" << output << " <= ap_start;" << std::endl;
        }
    }
    
    out << std::endl << "-- ======= DATA FLOW EQUATIONS OUT =======" << std::endl;
    
    out << container._conversionOut << std::endl;
    out << std::endl << "end DSP;" << std::endl;
    return out;
}

//fill _mappings and _output_mappings

void VhdlCodeContainer::connect(const Vertex& source, const Vertex& target, int lag)
{
    // Find the source and target identifiers
    size_t source_hash = source.node_hash;
    size_t target_hash = target.node_hash;

    //Define the vhdl type
    VhdlType sig_type;
    switch (source.nature) {
        case Nature::kInt: sig_type = VhdlType(VhdlInnerType::Integer); break;
        case Nature::kAny:
        case Nature::kReal: sig_type = VhdlType((gGlobal->gVHDLFloatEncoding ? VhdlInnerType::RealFloat : VhdlInnerType::SFixed), 8, -23); break;
    }

    // Create the lag component if necessary, returning the resulting id or 0
    size_t registers_id;
    if (target.is_output() && !target.is_recursive()) {
        registers_id = lag ? generateRegisterSeries(lag, sig_type, source_hash) : 0;
    } else {
        registers_id = lag ? generateRegisterSeries(lag, sig_type) : 0;
    }
    //registers_id=0 is kept for ap_start
    if (!(registers_id==0)) {
        _register_series[registers_id].source = std::make_optional(source_hash);
    }
    if (!(target.is_output() && !target.is_recursive())) {
        _mappings.find(target_hash)->second.push_back({ source_hash, registers_id });
    }

    // Finally, add the connection in the correct mappings table
    if (target.is_output() && !target.is_recursive()) {
        if(lag){
            _output_mappings.push_back({source_hash, sig_type, registers_id});
        } else {
            _output_mappings.push_back({source_hash, sig_type});
        }
    }
}

// Generate input mappings with the right conversion
void VhdlCodeContainer::convertIn(VhdlType type, int i){
    if(type.type == VhdlInnerType::SFixed){
        _signals << std::endl << "signal data_in_" << i <<" : sfixed (0 downto -23);" << std::endl;
        _signals << "signal aud_in_" << i <<" : sfixed (8 downto -23);" << std::endl;
        _conversionIn << std::endl << "data_in_" << i <<" <= to_sfixed (audio_in_" << i << ", 0, -23);" << std::endl;
        _conversionIn << "aud_in_" << i <<" <= resize (data_in_" << i << ", 8, -23);" << std::endl;
    } else if(type.type == VhdlInnerType::RealFloat){
        _signals << "signal aud_in_" << i <<" : float (8 downto -23);" << std::endl;
        _conversionIn << std::endl << "aud_in_" << i <<" <= to_float (\"00000000\" & audio_in_" << i << ", 8, 23);" << std::endl;
    }
}

// Generate output mappings with the right conversions
void VhdlCodeContainer::convertOut(){

    for (size_t output = 0; output < _num_outputs; ++output) {
        size_t output_mapping;
        if (output < _output_mappings.size()) {
            output_mapping = output;
        } else {
            output_mapping = _output_mappings.size() - 1;
        }

        auto element = _output_mappings[output_mapping][0];
        auto element2 = _output_mappings[output_mapping][1];
        auto element3 = _output_mappings[output_mapping][2];
        auto source_hash = std::get_if<size_t>(&element);
        auto type = std::get_if<VhdlType>(&element2);

        if(std::holds_alternative<size_t>(element3)) {
            auto registers_id = std::get<size_t>(element3);
            if (type->type == VhdlInnerType::SFixed){
                _signals << std::endl << "signal aud_out_" << output << " : std_logic_vector (31 downto 0);" << std::endl;
                _conversionOut << "aud_out_" << output << " <= " << " to_Std_Logic_Vector (" << "registers_" << registers_id << ");" << std::endl;
                _conversionOut << "audio_out_" << output << " <= " << " aud_out_" << output << "(23 downto 0);" << std::endl;
            } else if(type->type == VhdlInnerType::Integer){
                _conversionOut << "audio_out_" << output << " <= " << " conv_Std_Logic_Vector (" << "registers_" << registers_id << ", 24);" << std::endl;
            } else if(type->type == VhdlInnerType::RealFloat){
                _signals << std::endl << "signal aud_out_" << output << " : std_logic_vector (31 downto 0);" << std::endl;
                _conversionOut << "aud_out_" << output << " <= " << " to_Std_Logic_Vector (" << "registers_" << registers_id << ");" << std::endl;
                _conversionOut << "audio_out_" << output << " <= " << " aud_out_" << output << "(23 downto 0);" << std::endl;
            }
        } else if(_signal_identifier.count(*source_hash) != 0) {
            auto source_id = _signal_identifier.at(*source_hash);

            if (type->type == VhdlInnerType::SFixed){
                _signals << std::endl << "signal aud_out_" << output << " : std_logic_vector (31 downto 0);" << std::endl;
                _conversionOut << "aud_out_" << output << " <= " << " to_Std_Logic_Vector (" << source_id << ");" << std::endl;
                _conversionOut << "audio_out_" << output << " <= " << " aud_out_" << output << "(23 downto 0);" << std::endl;
            } else if(type->type == VhdlInnerType::Integer){
                _conversionOut << "audio_out_" << output << " <= " << " conv_Std_Logic_Vector (" << source_id  << ", 24);" << std::endl;
            } else if(type->type == VhdlInnerType::RealFloat){
                _signals << std::endl << "signal aud_out_" << output << " : std_logic_vector (31 downto 0);" << std::endl;
                _conversionOut << "aud_out_" << output << " <= " << " to_Std_Logic_Vector (" << source_id << ");" << std::endl;
                _conversionOut << "audio_out_" << output << " <= " << " aud_out_" << output << "(23 downto 0);" << std::endl;
            }
        }
    }
}




void VhdlCodeContainer::fill_delays(size_t delay_hash,int constant){
    _delays.insert({delay_hash,constant});
}


//
//*some useful fonctions*/
//



std::string entityTypeFromName(const std::string& name) {
    std::string entity_type;
    auto last_underscore = name.find_last_of('_');
    for (size_t i = 0; i < last_underscore && i < name.size(); ++i) {
        entity_type.push_back(name[i]);
    }
    return entity_type;
}

std::string VhdlType::to_string() const
{
    std::stringstream s;
    switch (type) {
        case VhdlInnerType::Bit: s << "Bit"; break;
        case VhdlInnerType::BitVector: s << "BitVector"; break;
        case VhdlInnerType::Boolean: s << "Boolean"; break;
        case VhdlInnerType::BooleanVector: s << "BooleanVector"; break;
        case VhdlInnerType::Integer: s << "Integer"; break;
        case VhdlInnerType::IntegerVector: s << "IntegerVector"; break;
        case VhdlInnerType::Natural: s << "Natural"; break;
        case VhdlInnerType::Positive: s << "Positive"; break;
        case VhdlInnerType::Character: s << "Character"; break;
        case VhdlInnerType::String: s << "String"; break;
        case VhdlInnerType::RealFloat: s << "Real"; break;
        case VhdlInnerType::StdLogic: s << "Logic"; break;
        case VhdlInnerType::StdLogicVector: s << "LogicVector"; break;
        case VhdlInnerType::StdULogic: s << "ULogic"; break;
        case VhdlInnerType::StdULogicVector: s << "ULogicVector"; break;
        case VhdlInnerType::Unsigned: s << "Unsigned"; break;
        case VhdlInnerType::Signed: s << "Signed"; break;
        case VhdlInnerType::UFixed: s << "UFixed"; break;
        case VhdlInnerType::SFixed: s << "SFixed"; break;
        default: break;
    }
    if (msb != lsb) {
        s << (msb < 0 ? "m" : "") << (msb < 0 ? -msb : msb) << "_" << (lsb < 0 ? "m" : "") << (lsb < 0 ? -lsb : lsb);
    }
    return s.str();
}



//
//*fonctions to output variables as vhdl code*/
//


std::string VhdlCodeContainer::entityName(const std::string& name, VhdlType type) const
{
    std::stringstream ss;
    ss << name << type.to_string();
    return ss.str();
}

std::ostream& operator<<(std::ostream& out, const VhdlType& type) {
    switch (type.type) {
        case VhdlInnerType::Bit: out << "bit"; break;
        case VhdlInnerType::BitVector: out << "bit_vector"; break;
        case VhdlInnerType::Boolean: out << "boolean"; break;
        case VhdlInnerType::BooleanVector: out << "boolean_vector"; break;
        case VhdlInnerType::Integer: out << "integer"; break;
        case VhdlInnerType::IntegerVector: out << "integer_vector"; break;
        case VhdlInnerType::Natural: out << "natural"; break;
        case VhdlInnerType::Positive: out << "positive"; break;
        case VhdlInnerType::Character: out << "character"; break;
        case VhdlInnerType::String: out << "string"; break;
        case VhdlInnerType::RealFloat: out << "float"; break;
        case VhdlInnerType::StdLogic: out << "std_logic"; break;
        case VhdlInnerType::StdLogicVector: out << "std_logic_vector"; break;
        case VhdlInnerType::StdULogic: out << "std_ulogic"; break;
        case VhdlInnerType::StdULogicVector: out << "std_ulogic_vector"; break;
        case VhdlInnerType::Unsigned: out << "unsigned"; break;
        case VhdlInnerType::Signed: out << "signed"; break;
        case VhdlInnerType::UFixed: out << "ufixed"; break;
        case VhdlInnerType::SFixed: out << "sfixed"; break;
        default: break;
    }
    if (type.msb != type.lsb) {
        out << '(' << type.msb << " downto " << type.lsb << ')';
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const VhdlPort& port)
{
   out << port.name << " : " << port.mode << " " << port.type;
   return out;
}

std::ostream& operator<<(std::ostream& out, const PortMode& port) {
    switch (port) {
        case PortMode::Input: out << "in"; break;
        case PortMode::Output: out << "out"; break;
        case PortMode::InOut: out << "inout"; break;
        case PortMode::Buffer: out << "buffer"; break;
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const VhdlValue& value) {
    std::ostringstream oss;
   switch (value.vhdl_type.type) {
       case VhdlInnerType::Integer:
       case VhdlInnerType::Natural:
       case VhdlInnerType::Positive:
       case VhdlInnerType::Signed:
       case VhdlInnerType::Unsigned: out << value.value.integer; break;

       case VhdlInnerType::RealFloat: out << "to_float(" << value.value.real << ", " << value.vhdl_type.msb << ", " << -value.vhdl_type.lsb << ")"; break;

       case VhdlInnerType::UFixed: out << "to_ufixed(" << value.value.real << ", " << value.vhdl_type.msb << ", " << value.vhdl_type.lsb << ")"; break;
       case VhdlInnerType::SFixed:
            oss << value.value.real;
            if(oss.str().find("e") != std::string::npos && oss.str().find(".") == std::string::npos){
                // add a .0 before e
                out << "to_sfixed(" << std::scientific << std::setprecision(1) << value.value.real << ", " << value.vhdl_type.msb << ", " << value.vhdl_type.lsb << ")"; 
            } else {
                out << "to_sfixed(" << value.value.real << ", " << value.vhdl_type.msb << ", " << value.vhdl_type.lsb << ")"; 
            }
            
            break;

       case VhdlInnerType::StdLogic: out << (value.value.boolean ? "'1'" : "'0'"); break;
       default: {
            std::cerr << __FILE__ << ":" << __LINE__ << " ASSERT : values of type " << value.vhdl_type << " are not yet implemented" << std::endl;
            faustassert(false);
       }
   }

   return out;
}


//
//*fonctions to fill the VhdlCodeBlocks with the code corresponding to each node*/
//


void VhdlCodeContainer::register_component(const Vertex& component, std::optional<int> cycles_from_input)
{
    if (!component.is_output()) {
        _mappings.insert({component.node_hash, {}});
    }

    Tree     sig = component.signal;
    int      i;
    int64_t  i64;
    double   r;
    Tree     x, y, l, cur, min, max, step, type, file, name;

    xtended* user_data = (xtended*)getUserData(sig);
    VhdlType sig_type;
    switch (component.nature) {
        case Nature::kInt:
            sig_type = VhdlType(VhdlInnerType::Integer);
            break;
        case Nature::kAny:
        case Nature::kReal:
            sig_type =
                VhdlType((gGlobal->gVHDLFloatEncoding ? VhdlInnerType::RealFloat : VhdlInnerType::SFixed), 8, -23);
    }
    
    // TODO: Add support for custom operators
    if (user_data) {
        // TODO: Find a more general way to handle user signals
        if (strcmp(user_data->name(), "fmod") == 0) {
            generateBinaryOperator(component.node_hash, SOperator::kRem, sig_type);
        } else if(strcmp(user_data->name(), "sin") == 0) {
            generateSineOperator(component.node_hash, sig_type);
        }
    } else if (isSigInt(sig, &i)) {
        generateConstant(component.node_hash, VhdlValue(i));
    } else if (isSigInt64(sig, &i64)) {
        generateConstant(component.node_hash, VhdlValue(i64));
    } else if (isSigReal(sig, &r)) {
        generateConstant(component.node_hash, VhdlValue(r));
    } else if (isSigInput(sig, &i) && component.is_recursive()) {
        // Recursive inputs do not generate anything
    } else if (isSigInput(sig, &i)) {
        // For inputs, we simply need to map the hash of the vertex to the correct identifier
        _signal_identifier.insert({component.node_hash, std::string("aud_in_") + std::to_string(i)});
        convertIn(sig_type,i);
    } else if (isSigOutput(sig, &i, x) && component.is_recursive()) {
        // For recursive outputs, we generate a one-sample delay as well
        // as a series of registers from ap_start to the write_enable signal
        // of the storage medium
        generateOneSampleDelay(component.node_hash, sig_type, *cycles_from_input);
    } else if (isSigOutput(sig, &i, x)) {
       // Normal outputs do not generate anything 
    } else if (isSigDelay1(sig, x)) {
        if(_delays.find(component.node_hash) != _delays.end()){
            if(_delays.at(component.node_hash) != 0){
               generateDelay(component.node_hash, sig_type, *cycles_from_input); 
            }else{
                generateBypass(component.node_hash, sig_type);
            }
        }else{
            generateBypass(component.node_hash, sig_type);
        } 
    } else if (isSigDelay(sig, x, y)) {
        if(_delays.find(component.node_hash) != _delays.end()){
            if(_delays.at(component.node_hash) != 0){
               generateDelay(component.node_hash, sig_type, *cycles_from_input); 
            }else{
                generateBypass(component.node_hash, sig_type);
            }
        }else{
            generateBypass(component.node_hash, sig_type);
        } 
    } else if (isSigBinOp(sig, &i, x, y)) {
        generateBinaryOperator(component.node_hash, i, sig_type);
    //new operators
    } else if (isSigFloatCast(sig, x)) {
        generateFloatCast(component.node_hash, sig_type);
    } else if (isSigIntCast(sig, x)) {
        generateIntCast(component.node_hash, sig_type);
    } else if (isRec(sig, x, y)) {
        generateBypass(component.node_hash, sig_type);
    } else if (isSigFConst(sig, type, name, file)) {
        generateFConstant(component.node_hash);
        return;
    }
    // TODO: implement missing operators, see the original SignalVisitor implementation

    // ca ne rentre pas ici car ca rentre dans le user_data du premier if car on le fait avec xtended
    else if(isSigSin(sig)) {
        generateSineOperator(component.node_hash, sig_type);
    }

    else if (isNil(sig)) {
    } else {
        std::cerr << __FILE__ << ":" << __LINE__ << " ASSERT : tried generating code for a non-implemented signal type : " << *sig << std::endl;
        faustassert(false);
    }
    
}

void VhdlCodeContainer::generateSineOperator(size_t hash, VhdlType type)
{
    std::string entity_name = entityName("Sin", type);
    int instance_identifier;
    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end())
    {
        _declared_entities.insert({entity_name, 0});
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
            _entities << "use work.fixed_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << std::endl << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "data_in_0: in " << type << ";" << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;

        _entities.open_block();
        _entities << "component SinCos24 is" << std::endl;
        _entities << "port (input : in   sfixed(1 downto -22) := to_sfixed(0, 1, -22);   --std_logic_vector(23 downto 0);" << std::endl;
        _entities.open_block();
        _entities << "SIN : out  sfixed(0 downto -23);" << std::endl;
        _entities << "COS : out  sfixed(0 downto -23));" << std::endl;
        _entities.close_block();
        _entities << "end component;" << std::endl;
        _entities << "signal SIN_out : sfixed(0 downto -23);" << std::endl;
        _entities << "signal SIN_in : sfixed(1 downto -22);" << std::endl;
        _entities << "constant inv_pi : sfixed(8 downto -23) := to_sfixed(0.31830988618, 8, -23);" << std::endl;
        _entities << "signal data_ready : sfixed(8 downto -23);" << std::endl;
        _entities.close_block();

        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "process(clock)" << std::endl;
        _entities.open_block();
        _entities << "variable data_mod : sfixed(8 downto -23);" << std::endl;
        _entities << "variable data_scaled : sfixed(8 downto -23);" << std::endl;
        _entities << "variable data_done : sfixed(8 downto -23);" << std::endl;
        if(gGlobal->gVHDLFloatEncoding){
            _entities << "variable converted_data_in_0 : sfixed(8 downto -23);" << std::endl;
        }
        _entities.close_block();
        _entities << "begin" << std::endl;
        _entities.open_block();
        if(gGlobal->gVHDLFloatEncoding){
            _entities << "converted_data_in_0 := to_sfixed(data_in_0, 8, -23);" << std::endl;
            _entities << "data_scaled := resize(converted_data_in_0 * inv_pi , 8, -23);  -- on divise l'argument du sin par pi car le sin de flopoco est sin(pi*X)" << std::endl;
        } else {
            _entities << "data_scaled := resize(data_in_0 * inv_pi , 8, -23);  -- on divise l'argument du sin par pi car le sin de flopoco est sin(pi*X)" << std::endl;
        }
        _entities << "data_mod := resize((data_scaled mod to_sfixed(2, 8, -23)), 8, -23);" << std::endl;
        
        _entities << "if (data_mod >= to_sfixed(1, 8, -23)) then" << std::endl;
        _entities.open_block();
        _entities << "data_done := resize((data_mod - to_sfixed(2, 8, -23)), 8, -23);" << std::endl;
        _entities.close_block();
        _entities << "else" << std::endl;
        _entities.open_block();
        _entities << "data_done := data_mod;" << std::endl;
        _entities.close_block();
        _entities << "end if;" << std::endl;
        

        _entities << "data_ready <= data_done;" << std::endl;
        _entities.close_block();
        _entities << "end process;" << std::endl;
        _entities << "SIN_in <= resize(data_ready, 1, -22);" << std::endl;
        _entities << "pm_SinCos24_0 : SinCos24" << std::endl;
        _entities << "port map (" << std::endl;
        _entities.open_block();
        _entities << "input => SIN_in," << std::endl;
        _entities << "SIN => SIN_out" << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        if(gGlobal->gVHDLFloatEncoding){
            _entities << "data_out <= to_float(resize(SIN_out, 8, -23), 8, 23);" << std::endl;
        } else {
            _entities << "data_out <= resize(SIN_out, 8, -23);" << std::endl;
        }
        _entities.close_block();
        
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "data_in_0: in " << type << ";" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(type);
    port_type.insert({hash,vector});
}

void VhdlCodeContainer::generateBinaryOperator(size_t hash, int kind, VhdlType type)
{
    std::string operator_name;
    std::string operator_symbol = gBinOpTable[kind]->fName;
    switch (kind) {
        case SOperator::kAdd: { operator_name = "Add"; break; }
        case SOperator::kSub: { operator_name = "Sub"; break; }
        case SOperator::kMul: { operator_name = "Mul"; break; }
        case SOperator::kDiv: { operator_name = "Div"; break; }
        case SOperator::kRem: { operator_name = "Rem"; operator_symbol = "mod"; break; }
        case SOperator::kLsh: { operator_name = "LeftShift"; break; }
        case SOperator::kARsh: { operator_name = "ArithRightShift"; break; }
        case SOperator::kLRsh: { operator_name = "LogicRightShift"; break; }
        case SOperator::kGT: { operator_name = "GreaterThan"; break; }
        case SOperator::kLT: { operator_name = "LesserThan"; break; }
        case SOperator::kGE: { operator_name = "GreaterEqual"; break; }
        case SOperator::kLE: { operator_name = "LesserEqual"; break; }
        case SOperator::kEQ: { operator_name = "Equal"; break; }
        case SOperator::kNE: { operator_name = "NotEqual"; break; }
        case SOperator::kAND: { operator_name = "And"; break; }
        case SOperator::kOR: { operator_name = "Or"; break; }
        case SOperator::kXOR: { operator_name = "Xor"; break; }
    }
    
    std::string entity_name = entityName(operator_name, type);

    int instance_identifier;
    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end())
    {
        _declared_entities.insert({entity_name, 0});
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << std::endl << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "data_in_0: in " << type << ";" << std::endl;
        _entities << "data_in_1: in " << type << ";" << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;
        if (kind >= SOperator::kGT && kind <= SOperator::kXOR) {
            _entities.open_block();
            _entities << "signal bool: boolean;" << std::endl;
            _entities.close_block();
            _entities << std::endl<< "begin"  << std::endl ;
            _entities.open_block();
            _entities << " bool <= data_in_0 "<< operator_symbol << " data_in_1;" << std::endl;
            _entities << "data_out <= 1 when bool = true else "  << std::endl;
            _entities << "0; " << std::endl;
            _entities.close_block();
        }else if (kind >= SOperator::kDiv && kind <= SOperator::kRem){
            _entities << "begin" << std::endl;
            _entities.open_block();
            if(type.type == VhdlInnerType::SFixed){
                _entities << "data_out <= to_sfixed(0, 8, -23) when data_in_1 <= to_sfixed(0, 8, -23) else " << std::endl;
                _entities.open_block();
                _entities << "resize(data_in_0 " << operator_symbol << " data_in_1 , 8, -23);" << std::endl;

            }else if(type.type == VhdlInnerType::RealFloat){
                _entities << "data_out <= to_float(0, 8, 23) when data_in_1 <= to_float(0, 8, 23) else " << std::endl;
                _entities.open_block();
                _entities << "resize(data_in_0 " << operator_symbol << " data_in_1 , 8, 23);" << std::endl;
            }else{
                _entities << "data_out <= 0 when data_in_1 = 0 else " << std::endl;
                _entities.open_block();
                _entities << "data_in_0 " << operator_symbol << " data_in_1;" << std::endl;
            }
            _entities.close_block();
            _entities.close_block();
        }else{
            _entities << "begin" << std::endl;
            _entities.open_block();
            if(type.type == VhdlInnerType::SFixed){
                _entities << "data_out <= resize(data_in_0 " << operator_symbol << " data_in_1 , 8, -23);" << std::endl;
            }else{
                _entities << "data_out <= data_in_0 " << operator_symbol << " data_in_1;" << std::endl;
            }
            _entities.close_block();
        }
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "data_in_0: in " << type << ";" << std::endl;
        _components << "data_in_1: in " << type << ";" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(type);
    vector.push_back(type);
    port_type.insert({hash,vector});

}

void VhdlCodeContainer::generateOneSampleDelay(size_t hash, VhdlType type, int cycles_from_input)
{
    std::string entity_name = entityName("OneSampleDelay", type);

    int instance_identifier;
    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end()) {
        _declared_entities.insert({entity_name, 0});
	
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "write_enable: in std_logic;" << std::endl;
        _entities << "data_in_0: in " << type << ";" << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;
        _entities << "signal data : " << type << ";" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "process (clock, reset)" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "if (reset = '0') then" << std::endl;
        _entities.open_block();
        _entities << "data <= " << VhdlValue(type) << ";" << std::endl;
        _entities.close_block();
        _entities << "elsif (clock'event and clock = '1') then" << std::endl;
        _entities.open_block();
        _entities << "if (write_enable = '1') then" << std::endl;
        _entities.open_block();
        _entities << "data <= data_in_0;" << std::endl;
        _entities.close_block();
        _entities << "end if;" << std::endl;
        _entities.close_block();
        _entities << "end if;" << std::endl;
        _entities.close_block();
        _entities << "end process;" << std::endl;
        _entities << "data_out <= data;" << std::endl;
        _entities.close_block();
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "write_enable: in std_logic;" << std::endl;
        _components << "data_in_0: in " << type << ";" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    // The write_enable port needs to be connected to a series of registers that runs parallel to the
    // currently computed sample, so that its value is '1' when the result to be saved is ready.
    if(cycles_from_input == 0) {
        // If the delay is 0, we do not need to generate any registers, the result is ready immediately, so we will connect the write enable to the ap_start signal
        _one_sample_delay_mappings.insert({hash, -1});
    } else {
        auto registers_id = generateRegisterSeries(cycles_from_input, VhdlType(VhdlInnerType::StdLogic));
        _one_sample_delay_mappings.insert({hash, registers_id});
    }

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(type);
    port_type.insert({hash,vector});
}

void VhdlCodeContainer::generateConstant(size_t hash, VhdlValue value)
{
    int instance_identifier;
    auto entry = _declared_entities.find("Constant");
    if (entry == _declared_entities.end())
    {
        _declared_entities.insert({"Constant", 0});
        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }
    std::string signal_identifier = std::string("constant_") + std::to_string(instance_identifier);
    if(value.vhdl_type.type == VhdlInnerType::RealFloat) {
        _signals << "signal " << signal_identifier << " : " << value.vhdl_type << " := " << "to_float(\"000000\" & to_slv(to_float(" << value.value.real << ", 2, 23)), 8, 23);" << std::endl;
    } else {
        _signals << "signal " << signal_identifier << " : " << value.vhdl_type << " := " << value << ";" << std::endl;
    }
    _signal_identifier.insert({hash, signal_identifier});
}

void VhdlCodeContainer::generateFConstant(size_t hash)
{
    int instance_identifier;
    auto entry = _declared_entities.find("FConstant");
    if (entry == _declared_entities.end())
    {
        _declared_entities.insert({"FConstant", 0});
        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }
    std::string signal_identifier = std::string("constant_") + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : integer := 48000 ;" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});
}

void VhdlCodeContainer::generateFloatCast(size_t hash, VhdlType type)
{
    std::string entity_name = entityName("CastFloat", type);

    int instance_identifier;

    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end()) {
        // TODO: right now, this is just a bypass entity made for basic programs to compile
        _declared_entities.insert({entity_name, 0});
	
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "data_in_0: in integer; " << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "data_out <= to_sfixed(data_in_0, 8, -23);" << std::endl;
        _entities.close_block();
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "data_in_0: in integer;" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(VhdlInnerType::Integer);
    port_type.insert({hash,vector});
}

void VhdlCodeContainer::generateIntCast(size_t hash, VhdlType type)
{
    std::string entity_name = entityName("CastInt", type);

    int instance_identifier;

    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end()) {
        // TODO: right now, this is just a bypass entity made for basic programs to compile
        _declared_entities.insert({entity_name, 0});
	
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "data_in_0: in sfixed (8 downto -23); " << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "data_out <= to_integer(data_in_0);" << std::endl;
        _entities.close_block();
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "data_in_0: in sfixed (8 downto -23);" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(VhdlInnerType::SFixed);
    port_type.insert({hash,vector});
}

size_t VhdlCodeContainer::generateRegisterSeries(int n, VhdlType type, std::optional<size_t> source)
{
    std::string entity_name = entityName("RegisterSeries", type);

    int instance_identifier;
    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end())
    {
        _declared_entities.insert({entity_name, 0});
    
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << std::endl << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "generic (" << std::endl;
        _entities.open_block();
        _entities << "n: natural := " << n << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "data_in: in " << type << ";" << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "type register_array is array(0 to n - 1) of " << type << ";" << std::endl;
        
        if (type.type == VhdlInnerType::StdLogic) {
            _entities << "signal registers: register_array := (others => '0');" << std::endl;
        }else if (type.type == VhdlInnerType::Integer){
            _entities << "signal registers: register_array := (others => 0);" << std::endl;
        }else{
            _entities << "signal registers: register_array := (others => (others => '0'));" << std::endl;
        }
        _entities.close_block();
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "process (clock, reset)" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "if reset = '0' then" << std::endl;
        _entities.open_block();
        if (type.type == VhdlInnerType::StdLogic) {
            _entities << "registers <= (others => '0');" << std::endl;
        }else if (type.type == VhdlInnerType::Integer){
            _entities << "registers <= (others => 0);" << std::endl;
        }else{
            _entities << "registers <= (others => (others => '0'));" << std::endl;
        }

        _entities.close_block();
        _entities << "elsif rising_edge(clock) then" << std::endl;
        _entities.open_block();
        _entities << "for i in n - 1 downto 1 loop" << std::endl;
        _entities.open_block();
        _entities << "registers(i) <= registers(i - 1);" << std::endl;
        _entities.close_block();
        _entities << "end loop;" << std::endl;
        _entities << "registers(0) <= data_in;" << std::endl;
        _entities.close_block();
        _entities << "end if;" << std::endl;
        _entities.close_block();
        _entities << "end process;" << std::endl;
        _entities << "data_out <= registers(n - 1);" << std::endl;
        _entities.close_block();
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "generic (" << std::endl;
        _components.open_block();
        _components << "n: natural := " << n << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "data_in: in " << type << ";" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    // Anyhow, we still instantiate a new signal
    std::string signal_name = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_name << " : " << type << " := " << VhdlValue(type) <<  ";" << std::endl;
    _register_series.push_back({ signal_name, source, n, type });
    return _register_series.size() - 1;

}

void VhdlCodeContainer::generateDelay(size_t hash, VhdlType type, int cycles_from_input)
{
    std::string entity_name = entityName("Delay", type);
 
    int instance_identifier;

    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end()) {
        // TODO: right now, this is just a bypass entity made for basic programs to compile
        _declared_entities.insert({entity_name, 0});
	
        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }
        
        _entities << std::endl << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "generic (" << std::endl;
        _entities.open_block();
        _entities << "n: natural := 1"  << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "clock_enable: in std_logic;" << std::endl;
        _entities << "data_in_0: in " << type << ";" << std::endl;
        _entities << "data_in_1: in integer;" << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << std::endl << "architecture Behavioral of " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "type register_array is array(0 to n - 1) of " << type << ";" << std::endl;
        
        if (type.type == VhdlInnerType::StdLogic) {
            _entities << "signal registers: register_array := (others => '0');" << std::endl;
        }else if (type.type == VhdlInnerType::Integer){
            _entities << "signal registers: register_array := (others => 0);" << std::endl;
        }else{
            _entities << "signal registers: register_array := (others => (others => '0'));" << std::endl;
        }
        _entities.close_block();
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "process (clock, reset)" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "if reset = '0' then" << std::endl;
        _entities.open_block();
        if (type.type == VhdlInnerType::StdLogic) {
            _entities << "registers <= (others => '0');" << std::endl;
        }else if (type.type == VhdlInnerType::Integer){
            _entities << "registers <= (others => 0);" << std::endl;
        }else{
            _entities << "registers <= (others => (others => '0'));" << std::endl;
        }

        _entities.close_block();
        _entities << "elsif rising_edge(clock) then" << std::endl;
        _entities.open_block();
        _entities << "if (clock_enable = '1') then" << std::endl;
        _entities.open_block();
        _entities << "for i in n - 1 downto 1 loop" << std::endl;
        _entities.open_block();
        _entities << "registers(i) <= registers(i - 1);" << std::endl;
        _entities.close_block();
        _entities << "end loop;" << std::endl;
        _entities << "registers(0) <= data_in_0;" << std::endl;
        _entities.close_block();
        _entities << "end if;" << std::endl;
        _entities.close_block();
        _entities << "end if;" << std::endl;
        _entities.close_block();
        _entities << "end process;" << std::endl;
        _entities << "data_out <= registers(n - 1);" << std::endl;
        _entities.close_block();
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "generic (" << std::endl;
        _components.open_block();
        _components << "n: natural := 1" << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "clock_enable: in std_logic;" << std::endl;
        _components << "data_in_0: in " << type << ";" << std::endl;
        _components << "data_in_1: in integer;" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    if(cycles_from_input > 0){
        // The write_enable port needs to be connected to a series of registers that runs parallel to the
        // currently computed sample, so that its value is '1' when the result to be saved is ready.
        auto registers_id = generateRegisterSeries(cycles_from_input, VhdlType(VhdlInnerType::StdLogic));
        _delays_mappings.insert({hash, registers_id});
    } else if(cycles_from_input == 0) {
        _delays_mappings.insert({hash, -1});
    }

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(type);
    vector.push_back(VhdlInnerType::Integer);
    port_type.insert({hash,vector});
    
}

void VhdlCodeContainer::generateBypass(size_t hash, VhdlType type)
{
    std::string entity_name = entityName("Bypass", type);

    int instance_identifier;

    auto entry = _declared_entities.find(entity_name);
    // If the generic entity is already declared, we do not redeclare it
    if (entry == _declared_entities.end()) {
        // TODO: right now, this is just a bypass entity made for basic programs to compile
        _declared_entities.insert({entity_name, 0});

        _entities << "library ieee;" << std::endl;
        _entities << "use ieee.std_logic_1164.all;" << std::endl;
        _entities << "use ieee.numeric_std.all;" << std::endl;
        _entities << "use ieee.std_logic_arith.all;" << std::endl;
        _entities << "use ieee.std_logic_signed.all;" << std::endl;
        _entities << "use work.fixed_float_types.all;" << std::endl;
        // Include the right package for real numbers encoding
        if (gGlobal->gVHDLFloatEncoding) {
            _entities << "use work.float_pkg.all;" << std::endl;
        } else {
            _entities << "use work.fixed_pkg.all;" << std::endl;
        }

        _entities << "entity " << entity_name << " is" << std::endl;
        _entities.open_block();
        _entities << "port (" << std::endl;
        _entities.open_block();
        _entities << "clock: in std_logic;" << std::endl;
        _entities << "reset: in std_logic;" << std::endl;
        _entities << "data_in_0: in " << type << ";" << std::endl;
        _entities << "data_out: out " << type << std::endl;
        _entities.close_block();
        _entities << ");" << std::endl;
        _entities.close_block();
        _entities << "end entity " << entity_name << ";" << std::endl;

        _entities << "architecture Behavioral of " << entity_name << " is" << std::endl;
        _entities << "begin" << std::endl;
        _entities.open_block();
        _entities << "data_out <= data_in_0;" << std::endl;
        _entities.close_block();
        _entities << "end architecture Behavioral;" << std::endl << std::endl;

        _components << "component " << entity_name << " is" << std::endl;
        _components.open_block();
        _components << "port (" << std::endl;
        _components.open_block();
        _components << "clock: in std_logic;" << std::endl;
        _components << "reset: in std_logic;" << std::endl;
        _components << "data_in_0: in " << type << ";" << std::endl;
        _components << "data_out: out " << type << std::endl;
        _components.close_block();
        _components << ");" << std::endl;
        _components.close_block();
        _components << "end component " << entity_name << ";" << std::endl << std::endl;

        instance_identifier = 0;
    } else {
        entry->second += 1;
        instance_identifier = entry->second;
    }

    std::string signal_identifier = entity_name + "_" + std::to_string(instance_identifier);
    _signals << "signal " << signal_identifier << " : " << type << " := " << VhdlValue(type) << ";" << std::endl;
    _signal_identifier.insert({hash, signal_identifier});

    //for each node we save their inputs' type to be able to convert at the mapping step
    std::vector<VhdlType> vector;
    vector.push_back(type);
    vector.push_back(type);
    port_type.insert({hash,vector});
}