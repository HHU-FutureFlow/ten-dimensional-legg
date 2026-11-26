import { useState } from 'react';
import { ChevronDown, ChevronUp } from 'lucide-react';
import type { NodeExplanation } from '../shared/model.ts';

export default function ExplanationCard({value}:{value:NodeExplanation}){
  const [expanded,setExpanded]=useState(false);
  return <div className={`node-explanation ${expanded?'expanded':''}`}>
    {([['接收',value.receives],['处理',value.process],['输出',value.produces]] as const).map(([label,text])=><div className="explanation-row" key={label}><strong>{label}</strong><p title={text}>{text||'尚未填写'}</p></div>)}
    {value.decisions.length>0&&<div className="explanation-row"><strong>判定</strong><ol>{(expanded?value.decisions:value.decisions.slice(0,1)).map((text,index)=><li key={index} title={text}>{text}</li>)}</ol></div>}
    <button type="button" className="explanation-toggle nodrag" onClick={e=>{e.stopPropagation();setExpanded(!expanded);}}>{expanded?'收起说明':value.decisions.length>1?`完整说明 · ${value.decisions.length} 条判定`:'展开完整说明'}{expanded?<ChevronUp size={12}/>:<ChevronDown size={12}/>}</button>
  </div>;
}
