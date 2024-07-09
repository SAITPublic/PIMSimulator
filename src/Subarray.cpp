/*********************************************************************************
 *  Copyright (c) 2010-2011, Elliott Cooper-Balis
 *                             Paul Rosenfeld
 *                             Bruce Jacob
 *                             University of Maryland
 *                             dramninjas [at] gmail [dot] com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

#include <memory>

#include "Subarray.h"
#include "BusPacket.h"

using namespace std;
using namespace DRAMSim;

Subarray::Subarray(ostream& simLog)
    : currentState(simLog), rowEntries(getConfigParam(UINT, "NUM_COLS")), dramsimLog(simLog)
{
    numCols = getConfigParam(UINT, "NUM_COLS");
}

/* The bank class is just a glorified sparse storage data structure
 * that keeps track of written data in case the simulator wants a
 * function DRAM model
 *
 * A vector of size NUM_COLS keeps a linked list of rows and their
 * associated values.
 *
 * write() adds an entry to the proper linked list or replaces the
 *     value in a row that was already written
 *
 * read() searches for a node with the right row value, if not found
 *     returns the tracer value 0xDEADBEEF
 *
 *    TODO: if anyone wants to actually store data, see the 'data_storage'
 *branch and perhaps try to merge that into master
 */

shared_ptr<Subarray::DataStruct> Subarray::searchForRow(unsigned row, shared_ptr<DataStruct> head)
{
    while (head != NULL)
    {
        if (head->row == row)
        {
            // found it
            return head;
        }
        // keep looking
        head = head->next;
    }
    // if we get here, didn't find it
    return NULL; 
}
//how about subarray model to array 
void Subarray::read(BusPacket* busPacket)
{
    shared_ptr<DataStruct> rowHeadNode = rowEntries[busPacket->column];
    shared_ptr<DataStruct> foundNode = NULL;
    cout<<"subarray read"<<" and row is "<<busPacket->row<<" and col is "<<busPacket->column<<endl;
    if ((foundNode = Subarray::searchForRow(busPacket->row, rowHeadNode)) == NULL)
    {
    }
    else  // found it
    {
        *(busPacket->data) = foundNode->data;
    }
}
//i'd like to use this logic same as subarray....
void Subarray::write(const BusPacket* busPacket)
{
    // TODO: move all the error checking to BusPacket so once we have a bus
    // packet,
    //            we know the fields are all legal
    cout<<"subarray write"<<" and row is "<<busPacket->row<<" and col is "<<busPacket->column<<endl;
    if (busPacket->column >= numCols)
    {
        ERROR("== Error - Bus Packet column " << busPacket->column << " out of bounds");
        exit(-1);
    }

    // head of the list we need to search
    shared_ptr<DataStruct> rowHeadNode = rowEntries[busPacket->column]; //such row groups that contains such column...
    shared_ptr<DataStruct> foundNode = NULL;
    if ((foundNode = Subarray::searchForRow(busPacket->row, rowHeadNode)) == NULL) //for certain row, no data 
    {
        // not found
        shared_ptr<DataStruct> newRowNode = make_shared<DataStruct>();
        // DataStruct* newRowNode = (DataStruct*)malloc(sizeof(DataStruct));

        // insert at the head for speed
        // TODO: Optimize this data structure for speedier lookups?
        newRowNode->row = busPacket->row;
        if (busPacket->data) //not null_bst_
            newRowNode->data = *(busPacket->data);
        newRowNode->next = rowHeadNode;
        rowEntries[busPacket->column] = newRowNode;
    }
    else
    {
        // found it, just plaster in the new data
        foundNode->data = *(busPacket->data);
        if (DEBUG_BANKS)
        {
            PRINTN(" -- Subarray " << busPacket->bank << " writing to physical address 0x" << hex
                               << busPacket->physicalAddress << dec << ":");
            busPacket->printData();
            PRINT("");
        }
    }
}
//nothing to change ildan...
