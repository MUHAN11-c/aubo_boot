/** Stable public error code shared by request and transport adapters. */
export class LinkError extends Error {
  constructor(public readonly code:string,message:string){super(message);this.name='LinkError';}
}
/** JavaScript permits throwing any value, including null and hostile coercions. */
export function errorText(value:unknown):string {
  try{return value instanceof Error?String(value.message):String(value);}
  catch{return 'Callback threw a value that cannot be converted to text';}
}
