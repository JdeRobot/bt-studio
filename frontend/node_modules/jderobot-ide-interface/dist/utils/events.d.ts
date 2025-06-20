export declare function subscribe(eventName: string, listener: (e: any) => void): void;
export declare function unsubscribe(eventName: string, listener: () => void): void;
export declare function publish(eventName: string, extra?: any): void;
