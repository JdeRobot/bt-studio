import React from 'react';
import Diagram, { useSchema, createSchema } from 'beautiful-react-diagrams';
import { Button } from 'beautiful-react-ui';

const CustomRender = ({ id, content, data, inputs, outputs }) => {

  // Determine the background color based on the id
  const backgroundColor = id === 'root-node' ? 'green' : '#cc9900';

  const childrenPort = outputs.find(port => port.props.id.startsWith('children'));
  const outputPorts = outputs.filter(port => !port.props.id.startsWith('children'));

  const parentPort = inputs.find(port => port.props.id.startsWith('parent-'));
  const inputPorts = inputs.filter(port => !port.props.id.startsWith('parent-'));

  const isRootNode = id === 'root-node';
  const marginTopStyle = !isRootNode ? '-10px' : '0px';

  return (
    <div style={{ display: 'flex', flexDirection: 'column', background: backgroundColor , borderRadius: '10px'}}>

      {/* Add the close button */}
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'end'}}>
        <div>
          {id !== 'root-node' && (
            <div 
              style={{ 
                width: '10px', 
                height: '10px', 
                borderRadius: '0 10px 0 0', 
                background: '#ff3300',
                cursor: 'pointer',
              }}
              onClick={() => data.onClick(id)}
            >
            </div>
          )}
        </div>
      </div>

      <div style={{ display: 'flex', flexDirection: 'row', marginTop: marginTopStyle}}>

        {/* Add the special parent node */}
        {parentPort && (
          <div style={{ display: 'flex', flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' }}>
            {React.cloneElement(parentPort, {
              style: {
                width: '10px',
                height: '10px',
                background: '#1B263B',
              }
            })}
          </div>
        )}
        
        <div role="button" style={{ padding: '10px' }}>
          {content}
        </div>

        {/* Add the special children node */}
        {childrenPort && (
          <div style={{ display: 'flex', flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between' }}>
            {React.cloneElement(childrenPort, {
              style: {
                width: '10px',
                height: '10px',
                background: "#1B263B",
                }
            })}
          </div>
        )}
      </div>

    </div>
  );
};

const initialSchema = createSchema({
  nodes: [
    {
      id: 'root-node',
      content: 'Root Node',
      coordinates: [150, 60],
      render: CustomRender,
      outputs: [
        { id: 'children', alignment: 'right'},
      ],
    },
  ]
});

let globalNodeId = 0;

const DiagramEditor = () => {

  // create diagrams schema
  const [schema, { onChange, addNode, removeNode }] = useSchema(initialSchema);

  const deleteNodeFromSchema = (id) => {
    const nodeToRemove = schema.nodes.find(node => node.id === id);
    removeNode(nodeToRemove);
  };

  const addNewNode = () => {

    globalNodeId++;
    const nextNodeId = `bt-node-${globalNodeId}`;
    const nextNode = {
      id: nextNodeId,
      content: `Node ${globalNodeId}`,
      coordinates: [
        schema.nodes[schema.nodes.length - 1].coordinates[0],
        schema.nodes[schema.nodes.length - 1].coordinates[1] + 150,
      ],
      // Remove the custom render to revert to default node appearance
      render: CustomRender,
      data: { onClick: deleteNodeFromSchema },
      inputs: [{ id: `parent-${nextNodeId}`, alignment: 'left'}],
      outputs: [{ id: `children-${nextNodeId}`, alignment: 'right'}],
    };
    
    console.log(nextNode)
    addNode(nextNode);
  };

  return (
    <div style={{ height: '80vh', width: '100%' }}> {/* Height and Width set explicitly */}
      <h2>Tree Editor</h2>
      <Button color="primary" icon="plus" onClick={addNewNode}>Add new node</Button>
      <Diagram schema={schema} onChange={onChange} />
    </div>
  );
};

export default DiagramEditor;
