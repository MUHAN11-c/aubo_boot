/** Public transport budgets have no dependency on a transport implementation.
 * Keep this declaration leaf independent of roslib: its 2.1.0 declaration entry
 * uses an extensionless ESM export which strict NodeNext consumers reject.
 */
export interface TransportLimits {
  /** UTF-8 JSON or binary CBOR bytes per frame and per assembled message. */
  maxMessageBytes:number;
  /** Aggregate retained fragment content, independent of message count. */
  maxFragmentBytes:number;maxFragmentMessages:number;maxFragmentsPerMessage:number;
  /** Deadline from first fragment; duplicates/extra parts never extend it. */
  fragmentTimeoutMs:number;
  /** Native socket bufferedAmount + next encoded send must fit this budget. */
  maxSendBufferedBytes:number;maxCborDepth:number;
}
export const DEFAULT_TRANSPORT_LIMITS:Readonly<TransportLimits>=Object.freeze({
  maxMessageBytes:16000000,maxFragmentBytes:32000000,maxFragmentMessages:8,
  maxFragmentsPerMessage:4096,fragmentTimeoutMs:30000,maxSendBufferedBytes:16000000,maxCborDepth:64,
});
