/* Offline JavaScript contract test; mock DOM/canvas, no browser or network. */
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2],'utf8');
const script=html.split('<script>')[1].split('</script>')[0];
const nodes=new Map(),callbacks=[],counts={strokes:0,text:0};
const context2d=new Proxy({}, {get(target,key){if(key==='stroke')return ()=>counts.strokes++;if(key==='fillText')return ()=>counts.text++;return target[key]??(()=>{});},set(target,key,value){target[key]=value;return true;}});
function element(id){if(!nodes.has(id))nodes.set(id,{id,width:1000,height:id==='map'?580:id==='health'?160:235,value:['view','window'].includes(id)?'all':id==='rate'?'20':id==='eventSearch'?'':'0',textContent:'',innerHTML:'',style:{},children:[],listeners:{},tagName:'DIV',getContext:()=>context2d,addEventListener(event,fn){this.listeners[event]=fn;},replaceChildren(...items){this.children=items;},add(item){this.children.push(item);},click(){this.onclick?.();},getBoundingClientRect:()=>({left:0,top:0,width:1000,height:235})});return nodes.get(id);}
const charts=['health','heading','error','gyro','accel','yaw','speedChart'];
const document={getElementById:element,querySelectorAll:()=>charts.map(element),addEventListener(){},activeElement:{tagName:'BODY'}};
const sandbox={document,console,Option:class{constructor(text,value){this.text=text;this.value=value;}},requestAnimationFrame:fn=>callbacks.push(fn)};
vm.createContext(sandbox);new vm.Script(script).runInContext(sandbox,{timeout:20000});
const evaluate=code=>vm.runInContext(code,sandbox,{timeout:20000});
assert.ok(element('title').textContent.includes('November 17'));
assert.ok(element('position').value.includes('Eastern'));
assert.ok(counts.strokes>0);assert.ok(counts.text>0);
element('next').click();assert.equal(Number(element('step').value),1);
element('prev').click();assert.equal(Number(element('step').value),0);
element('eventSearch').value='gap';element('eventSearch').oninput();assert.equal(element('events').children.length,3);
element('nextEvent').click();assert.ok(Number(element('step').value)>0);
assert.ok(element('facts').innerHTML.includes('Relative yaw / segment'));
element('zoomIn').click();assert.equal(element('zoomReset').textContent,'135%');element('zoomReset').click();
element('view').value='15';element('view').onchange();
element('window').value='120';element('window').onchange();assert.ok(evaluate('chartBounds()[1]-chartBounds()[0]')<=120.0001);
element('gyro').listeners.click({clientX:500});assert.ok(Number(element('step').value)>=0);
element('events').value=String(evaluate('samples.length-1'));element('events').onchange();assert.equal(Number(element('step').value),evaluate('samples.length-1'));
element('play').click();assert.equal(element('play').textContent,'Pause');assert.equal(Number(element('step').value),0);
callbacks.shift()(0);callbacks.shift()(1000);assert.ok(Number(element('step').value)>0);
element('play').click();assert.equal(element('play').textContent,'Play');
console.log(JSON.stringify({status:'PASS',playbackSamples:evaluate('samples.length'),events:evaluate('events.length'),controls:['step','event search','event jump','map zoom','local view','chart window','chart seek','end/restart','play/pause'],canvasCalls:counts},null,2));
