import { useRef } from 'react';
import { NodeResizer } from '@xyflow/react';
import Markdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import { NotebookPen } from 'lucide-react';
import type { BlockAppearance, Point } from '../shared/model.ts';
import { appearanceAttributes } from './blockAppearance.ts';

export function BoardMarkdown({text}:{text:string}){
  return <Markdown remarkPlugins={[remarkGfm]} skipHtml components={{
    a:({node,...props})=><a {...props} target="_blank" rel="noopener noreferrer" onClick={e=>e.stopPropagation()}/>,
    img:()=>null,
  }}>{text}</Markdown>;
}

export default function AnnotationCard({id,label,text,tone,appearance,selected,revision,resize}:{
  id:string;label:string;text:string;tone:string;appearance?:BlockAppearance;selected:boolean;revision:number;
  resize:(id:string,position:Point,width:number,height:number,baseRevision:number)=>void;
}){
  const base=useRef(revision);
  return <div className={`annotation-board tone-${tone} ${selected?'is-selected':''}`} {...appearanceAttributes(appearance)} data-testid={`graph-node-${id}`}>
    <NodeResizer isVisible={selected} minWidth={280} minHeight={180} maxWidth={2000} maxHeight={4000} onResizeStart={()=>{base.current=revision;}} onResizeEnd={(_,box)=>resize(id,{x:box.x,y:box.y},box.width,box.height,base.current)}/>
    <div className="board-header"><NotebookPen size={17}/><strong>{label}</strong><span>说明板</span></div>
    <div className="board-content nodrag nopan" onDoubleClick={e=>e.stopPropagation()}><BoardMarkdown text={text}/></div>
  </div>;
}
